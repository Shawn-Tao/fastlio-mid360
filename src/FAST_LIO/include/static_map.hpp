#pragma once

// Archive-only temporal occupancy policy. No ROS, PCL or changes to the live
// estimator/ikd-Tree. Negative evidence is ONLY an observed return's free ray.
#include "workspace_runtime.hpp"
#include <chrono>
#include <list>

namespace fastlio_runtime {
struct StaticMapOptions {
    bool enabled=true;
    bool confirmation_enabled=true, clearing_enabled=true;
    int min_observations=3, clear_observations=3;
    double observation_interval=0.1, confirmation_seconds=0.6, candidate_ttl=5.0;
    double clear_observation_interval=-1.0; // legacy YAML: inherit observation_interval
    double clear_seconds=0.4, clear_vote_ttl=3.0, clear_max_range=15.0, endpoint_margin=0.5, ray_clearance=0.025;
    std::size_t max_candidates=250000, max_frame_points=100000, max_rays=256, max_ray_steps=600;
    void validate(double voxel,std::size_t map_limit) const {
        for(double value:{candidate_ttl,clear_vote_ttl,
                          clear_max_range,endpoint_margin,ray_clearance})
            if(!std::isfinite(value) || value<=0) throw std::invalid_argument("static_map timing/ray values must be finite and positive");
        for(double value:{observation_interval,confirmation_seconds,clear_seconds})
            if(!std::isfinite(value) || value<0) throw std::invalid_argument("static_map intervals/spans must be finite and >= 0");
        if(!std::isfinite(clear_observation_interval) || (clear_observation_interval<0 && clear_observation_interval!=-1))
            throw std::invalid_argument("static_map.clear_observation_interval must be -1 (inherit) or >= 0");
        if(min_observations<1 || clear_observations<1 || !max_candidates || !max_frame_points || !max_rays || !max_ray_steps)
            throw std::invalid_argument("static_map counts must be positive");
        if(enabled && confirmation_enabled && candidate_ttl<=confirmation_seconds)
            throw std::invalid_argument("static_map.candidate_ttl must exceed confirmation_seconds");
        if(enabled && clearing_enabled && (clear_vote_ttl<=clear_seconds || endpoint_margin>=clear_max_range || ray_clearance>voxel*0.5))
            throw std::invalid_argument("static_map clearing requires vote_ttl > clear_seconds, endpoint_margin < clear_max_range and ray_clearance <= voxel_size/2");
        if(enabled && (!std::isfinite(voxel) || voxel<1e-4 || !map_limit))
            throw std::invalid_argument("static_map requires positive bounded voxel map; disable static_filter for legacy raw/unbounded mode");
    }
    double clear_interval() const { return clear_observation_interval==-1?observation_interval:clear_observation_interval; }
};

// Separate from admission and directly testable without ROS. Relaxing these
// limits does NOT improve scan-end ray-origin accuracy or change estimator health.
struct ClearMotionLimits {
    double max_position_std=0.10,max_speed=1.0,max_angular_speed=1.0,max_frame_gap=0.5;
    void validate() const {
        for(double v:{max_position_std,max_speed,max_angular_speed,max_frame_gap})
            if(!std::isfinite(v) || v<=0) throw std::invalid_argument("static_map clearing motion limits must be finite and positive");
    }
    const char* reason(double gap,double position_std,double speed,double angular,double peak_gyro) const {
        for(double v:{gap,position_std,speed,angular,peak_gyro})
            if(!std::isfinite(v) || v<0) return "invalid_motion";
        if(gap<=0 || gap>max_frame_gap) return "frame_gap";
        if(position_std>max_position_std) return "position_std";
        if(speed>max_speed) return "speed";
        if(angular>max_angular_speed) return "pose_angular_speed";
        if(peak_gyro>max_angular_speed) return "imu_peak_angular_speed";
        return "allowed";
    }
};

template<class Point> class StaticVoxelMap {
    struct Key {
        std::int64_t x,y,z;
        bool operator==(const Key& b) const { return x==b.x && y==b.y && z==b.z; }
    };
    struct Hash {
        std::size_t operator()(const Key& k) const {
            std::size_t h=std::hash<std::int64_t>{}(k.x);
            for(auto v:{k.y,k.z}) h^=std::hash<std::int64_t>{}(v)+0x9e3779b9+(h<<6)+(h>>2);
            return h;
        }
    };
    struct Candidate {
        Point point;
        double first,last,support;
        int hits;
        typename std::list<Key>::iterator age;
    };
    struct Occupied {
        Point point;
        double first_miss=0,last_miss=-1;
        int misses=0;
    };
    StaticMapOptions options_;
    double voxel_,stamp_=-std::numeric_limits<double>::infinity(),last_ms_=0;
    std::size_t limit_;
    VoxelMap<Point> legacy_;
    std::unordered_map<Key,Candidate,Hash> candidates_;
    std::unordered_map<Key,Occupied,Hash> occupied_;
    std::list<Key> age_; // one entry per candidate; O(1) refresh, never unbounded
    std::unordered_map<Key,Point,Hash> observations_;
    std::vector<Key> ray_keys_;
    std::uint64_t input_=0,invalid_=0,rejected_=0,candidate_rejected_=0,frame_rejected_=0;
    std::uint64_t promoted_=0,cleared_=0,expired_=0,skipped_clear_=0,bad_frames_=0,ray_steps_=0,limited_rays_=0;
    std::size_t last_rays_=0,last_steps_=0;
    std::uint64_t frames_=0;

    Key key(double x,double y,double z) const {
        return {std::int64_t(std::floor(x/voxel_)),std::int64_t(std::floor(y/voxel_)),std::int64_t(std::floor(z/voxel_))};
    }
    Key key(const Point& p) const { return key(p.x,p.y,p.z); }
    bool valid(const Point& p) const {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
            std::max({std::abs(double(p.x)),std::abs(double(p.y)),std::abs(double(p.z))})<=1e6;
    }
    double center_distance(const Point& p,const Key& k) const {
        const double x=p.x-(k.x+0.5)*voxel_,y=p.y-(k.y+0.5)*voxel_,z=p.z-(k.z+0.5)*voxel_;
        return x*x+y*y+z*z;
    }
    void representative(Point& old,const Point& p,const Key& k) {
        if(center_distance(p,k)<center_distance(old,k)) old=p;
    }
    void expire(double stamp) {
        // Bounded sweep of the LRU head, not of the historical/global map.
        std::size_t budget=options_.max_frame_points;
        while(budget-- && !age_.empty()) {
            const auto found=candidates_.find(age_.front());
            if(stamp-found->second.last<=options_.candidate_ttl) break;
            candidates_.erase(found); age_.pop_front(); ++expired_;
        }
    }
    bool hit_near(const Key& k) const {
        // A current hit (including an unconfirmed person) protects its immediate
        // neighbourhood. Do NOT confuse occlusion or endpoint uncertainty with air.
        for(int x=-1;x<=1;++x) for(int y=-1;y<=1;++y) for(int z=-1;z<=1;++z)
            if(observations_.count({k.x+x,k.y+y,k.z+z})) return true;
        return false;
    }
    void clear_ray(const Point& end,const Vector& origin,double stamp) {
        Vector direction{double(end.x)-origin[0],double(end.y)-origin[1],double(end.z)-origin[2]};
        const double range=std::hypot(std::hypot(direction[0],direction[1]),direction[2]);
        const double stop=std::min(options_.clear_max_range,range-options_.endpoint_margin);
        if(stop<=options_.endpoint_margin) return;
        for(double& value:direction) value/=range;
        Key cell=key(origin[0],origin[1],origin[2]);
        std::array<std::int64_t,3> indices{cell.x,cell.y,cell.z};
        std::array<int,3> step{};
        Vector next{},delta{};
        for(int a=0;a<3;++a) {
            step[a]=direction[a]>0?1:-1;
            if(std::abs(direction[a])<1e-12) next[a]=delta[a]=std::numeric_limits<double>::infinity();
            else {
                next[a]=((indices[a]+(step[a]>0?1:0))*voxel_-origin[a])/direction[a];
                delta[a]=voxel_/std::abs(direction[a]);
            }
        }
        double distance=0;
        std::size_t walked=0;
        for(;walked<options_.max_ray_steps && distance<stop;++walked) {
            cell={indices[0],indices[1],indices[2]}; ++last_steps_;
            // All frame endpoints were collected before any ray was processed.
            // Stop at a nearer return, even if it was not selected for ray casting.
            if(distance>0 && observations_.count(cell)) break;
            auto found=occupied_.find(cell);
            if(found!=occupied_.end() && stamp>found->second.last_miss &&
               stamp-found->second.last_miss>=options_.clear_interval()) {
                auto& item=found->second;
                const Vector p{double(item.point.x)-origin[0],double(item.point.y)-origin[1],double(item.point.z)-origin[2]};
                const double along=p[0]*direction[0]+p[1]*direction[1]+p[2]*direction[2];
                double perpendicular=0;
                for(int a=0;a<3;++a) perpendicular+=(p[a]-along*direction[a])*(p[a]-along*direction[a]);
                if(along>options_.endpoint_margin && along<stop &&
                   perpendicular<=options_.ray_clearance*options_.ray_clearance && !hit_near(cell)) {
                    if(stamp-item.last_miss>options_.clear_vote_ttl) item.misses=0;
                    if(item.misses==0) item.first_miss=stamp;
                    item.last_miss=stamp;
                    if(item.misses<options_.clear_observations) ++item.misses;
                    if(item.misses>=options_.clear_observations && stamp-item.first_miss>=options_.clear_seconds) {
                        occupied_.erase(found); ++cleared_;
                    }
                }
            }
            const int axis=next[0]<=next[1] && next[0]<=next[2]?0:(next[1]<=next[2]?1:2);
            distance=next[axis]; indices[axis]+=step[axis]; next[axis]+=delta[axis];
        }
        if(walked==options_.max_ray_steps && distance<stop) ++limited_rays_;
    }
public:
    StaticVoxelMap(double voxel,std::size_t limit,StaticMapOptions options={}):
        options_(options),voxel_(voxel),limit_(limit),legacy_(voxel,limit) { options_.validate(voxel,limit); }
    StaticVoxelMap(const StaticVoxelMap&)=delete;
    StaticVoxelMap& operator=(const StaticVoxelMap&)=delete;
    std::size_t frame_limit() const { return options_.enabled?options_.max_frame_points:std::numeric_limits<std::size_t>::max(); }
    bool confirmation_enabled() const { return options_.enabled && options_.confirmation_enabled; }
    bool clearing_enabled() const { return options_.enabled && options_.clearing_enabled; }

    // omitted counts endpoints the caller skipped before coordinate conversion.
    // It is a truncation, NOT a free/no-return ray. Node uses frame_limit().
    template<class Container> bool insert_frame(const Container& points,const Vector& origin,double stamp,bool allow_clear=true,std::size_t omitted=0) {
        const auto start=std::chrono::steady_clock::now();
        if(!std::isfinite(stamp) || stamp<0 || stamp<=stamp_ || (!options_.enabled && omitted) ||
           !std::all_of(origin.begin(),origin.end(),[](double v){return std::isfinite(v) && std::abs(v)<=1e6;})) {
            ++bad_frames_; return false;
        }
        stamp_=stamp; last_rays_=last_steps_=0; ++frames_;
        if(!options_.enabled) { for(const auto& p:points) legacy_.insert(p); }
        else {
            expire(stamp); observations_.clear(); ray_keys_.clear();
            std::size_t count=0;
            input_+=omitted; frame_rejected_+=omitted;
            bool truncated=omitted>0;
            for(const auto& p:points) {
                ++input_;
                if(count++>=options_.max_frame_points) { ++frame_rejected_; truncated=true; continue; }
                if(!valid(p)) { ++invalid_; continue; }
                const auto k=key(p);
                auto observed=observations_.emplace(k,p);
                if(observed.second) ray_keys_.push_back(k);
                else representative(observed.first->second,p,k);
            }
            for(const auto& k:ray_keys_) {
                const auto& p=observations_.at(k);
                auto old=occupied_.find(k);
                if(old!=occupied_.end()) {
                    representative(old->second.point,p,k);
                    old->second.misses=0; old->second.last_miss=-1; continue;
                }
                if(!options_.confirmation_enabled) {
                    if(occupied_.size()>=limit_) { ++rejected_; continue; }
                    occupied_.emplace(k,Occupied{p}); ++promoted_; continue;
                }
                auto found=candidates_.find(k);
                if(found==candidates_.end()) {
                    if(candidates_.size()>=options_.max_candidates) { ++candidate_rejected_; continue; }
                    age_.push_back(k);
                    found=candidates_.emplace(k,Candidate{p,stamp,stamp,stamp,1,std::prev(age_.end())}).first;
                } else {
                    auto& c=found->second;
                    if(stamp-c.last>options_.candidate_ttl) { c.point=p; c.first=c.support=stamp; c.hits=1; ++expired_; }
                    else if(stamp-c.support>=options_.observation_interval) {
                        if(c.hits<options_.min_observations) ++c.hits;
                        c.support=stamp;
                    }
                    c.last=stamp; representative(c.point,p,k); age_.splice(age_.end(),age_,c.age);
                }
                const auto& c=found->second;
                if(c.hits>=options_.min_observations && stamp-c.first>=options_.confirmation_seconds) {
                    if(occupied_.size()>=limit_) { ++rejected_; continue; }
                    occupied_.emplace(k,Occupied{c.point}); age_.erase(c.age); candidates_.erase(found); ++promoted_;
                }
            }
            // Discarded frame endpoints cannot act as occlusion guards. Such a
            // frame must NEVER contribute negative evidence.
            if(options_.clearing_enabled && allow_clear && !truncated) {
                const std::size_t rays=std::min(options_.max_rays,ray_keys_.size());
                for(std::size_t i=0;i<rays;++i) {
                    // Rotate a stratified subset instead of permanently casting
                    // the same few rays when point ordering is repeatable.
                    const auto index=(i*ray_keys_.size()/rays+frames_*73)%ray_keys_.size();
                    clear_ray(observations_.at(ray_keys_[index]),origin,stamp); ++last_rays_;
                }
                ray_steps_+=last_steps_;
            } else if(options_.clearing_enabled) ++skipped_clear_;
        }
        last_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        return true;
    }
    std::size_t size() const { return options_.enabled?occupied_.size():legacy_.size(); }
    std::size_t candidates() const { return candidates_.size(); }
    bool complete() const { return options_.enabled ? !(rejected_ || candidate_rejected_ || frame_rejected_) : legacy_.complete(); }
    std::uint64_t rejected() const { return options_.enabled?rejected_:legacy_.rejected(); }
    std::uint64_t invalid() const { return options_.enabled?invalid_:legacy_.invalid(); }
    std::uint64_t input() const { return options_.enabled?input_:legacy_.input(); }
    std::uint64_t cleared() const { return cleared_; }
    template<class Container> void snapshot_into(Container& out,std::size_t display_limit=0) const {
        if(!options_.enabled) { legacy_.snapshot_into(out,display_limit); return; }
        const auto stride=display_limit && size()>display_limit?(size()+display_limit-1)/display_limit:1;
        out.clear(); out.reserve((size()+stride-1)/stride);
        std::size_t i=0; for(const auto& cell:occupied_) if(i++%stride==0) out.push_back(cell.second.point);
    }
    std::string status_json() const {
        std::ostringstream out;
        out<<"{\"policy\":\"archive_temporal_rays_v2\",\"enabled\":"<<(options_.enabled?"true":"false")
           <<",\"confirmation_enabled\":"<<(confirmation_enabled()?"true":"false")
           <<",\"clearing_enabled\":"<<(clearing_enabled()?"true":"false")
           <<",\"confirmation_interval\":"<<options_.observation_interval
           <<",\"clear_interval\":"<<options_.clear_interval()
           <<",\"confirmed\":"<<size()<<",\"candidates\":"<<candidates_.size()<<",\"promoted\":"<<promoted_
           <<",\"cleared\":"<<cleared_<<",\"expired_candidates\":"<<expired_
           <<",\"candidate_capacity_rejected\":"<<candidate_rejected_<<",\"frame_points_rejected\":"<<frame_rejected_
           <<",\"clear_skipped_frames\":"<<skipped_clear_<<",\"invalid_frames\":"<<bad_frames_
           <<",\"last_rays\":"<<last_rays_<<",\"last_ray_steps\":"<<last_steps_
           <<",\"total_ray_steps\":"<<ray_steps_<<",\"step_limited_rays\":"<<limited_rays_
           <<",\"last_update_ms\":"<<last_ms_<<"}";
        return out.str();
    }
};
} // namespace fastlio_runtime
