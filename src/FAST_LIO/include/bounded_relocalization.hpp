#pragma once

// Dependency-free, bounded x/y/z/yaw registration for a stationary, upright
// platform. Coarse multi-start search is followed by trimmed point-to-point ICP.
// No claim of global optimality: ambiguous or poor matches must be rejected.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace fastlio_relocalization {
constexpr double pi = 3.14159265358979323846;
struct Point { double x = 0, y = 0, z = 0; };
struct Pose { double x = 0, y = 0, z = 0, yaw = 0; };
inline double angle(double a) { return std::remainder(a, 2 * pi); }
inline Point transform(const Point& p, const Pose& t) {
    const double c = std::cos(t.yaw), s = std::sin(t.yaw);
    return {c * p.x - s * p.y + t.x, s * p.x + c * p.y + t.y, p.z + t.z};
}
inline double squared_distance(const Point& a, const Point& b) {
    return (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z);
}
struct Options {
    Point center{};                       // IMU pose, not LiDAR/map centroid
    double radius = 3.0;                 // strict 3-D radius, metres
    double z_range = 0.5;                // also restrict height relative to center
    double xy_step = 0.75, z_step = 0.5, yaw_step_deg = 15.0;
    double voxel_size = 0.25, scan_range = 20.0;
    double coarse_distance = 1.0, icp_distance = 0.75, inlier_distance = 0.35;
    double min_overlap = 0.65, max_rmse = 0.18;
    double ambiguity_margin = 0.02;       // truncated RMS cost gap, metres
    double distinct_distance = 0.5, distinct_yaw_deg = 15.0;
    double max_seconds = 20.0;
    int min_points = 120, coarse_points = 256, max_points = 6000;
    int candidates = 24, iterations = 50;
    void validate() const {
        const std::array<double, 20> positive{{radius, xy_step, z_step, yaw_step_deg,
            voxel_size, scan_range, coarse_distance, icp_distance, inlier_distance,
            min_overlap, max_rmse, distinct_distance, distinct_yaw_deg, max_seconds,
            double(min_points), double(coarse_points), double(max_points),
            double(candidates), double(iterations), 1.0}};
        for (double v : positive)
            if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Relocalization parameters must be finite and positive");
        if (!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(center.z) ||
            !std::isfinite(z_range) || z_range < 0 || !std::isfinite(ambiguity_margin) || ambiguity_margin < 0 ||
            min_overlap > 1 || yaw_step_deg > 180 || max_points < min_points || coarse_points < 3 ||
            voxel_size < 1e-4 || min_points < 3 || max_points > 50000 || coarse_points > 2048 ||
            candidates > 128 || iterations > 200 || radius > 1000)
            throw std::invalid_argument("Invalid relocalization center, overlap, height, angle or point limits");
        const double positions = std::pow(2 * std::ceil(radius / xy_step) + 1, 2);
        const double heights = 2 * std::ceil(std::min(radius, z_range) / z_step) + 1;
        if (positions * heights * std::ceil(360 / yaw_step_deg) > 100000)
            throw std::invalid_argument("Relocalization search exceeds 100000 seeds; increase search steps");
    }
};
struct Result {
    bool accepted = false, converged = false;
    Pose pose{};
    double overlap = 0, rmse = std::numeric_limits<double>::infinity();
    double cost = std::numeric_limits<double>::infinity(), seconds = 0;
    int points = 0, inliers = 0;
    std::string reason = "insufficient_points";
};

// Balanced immutable kd-tree; safe for a search worker and main-thread checks.
class Tree {
    struct Node { Point p; int axis, left = -1, right = -1; };
    std::vector<Node> nodes_;
    int root_ = -1;
    static double coordinate(const Point& p, int a) { return a == 0 ? p.x : a == 1 ? p.y : p.z; }
    int build(std::vector<Point>& p, int begin, int end, int depth) {
        if (begin == end) return -1;
        int middle = begin + (end-begin)/2, axis = depth%3;
        std::nth_element(p.begin()+begin, p.begin()+middle, p.begin()+end,
            [axis](const Point& a, const Point& b) { return coordinate(a,axis) < coordinate(b,axis); });
        int n = static_cast<int>(nodes_.size());
        nodes_.push_back({p[middle], axis, -1, -1});
        int left = build(p,begin,middle,depth+1), right = build(p,middle+1,end,depth+1);
        nodes_[n].left = left; nodes_[n].right = right;
        return n;
    }
    void nearest(int n, const Point& q, double& d, Point& answer) const {
        if (n < 0) return;
        const auto& node = nodes_[n];
        double here = squared_distance(q,node.p);
        if (here < d) { d = here; answer = node.p; }
        double delta = coordinate(q,node.axis)-coordinate(node.p,node.axis);
        nearest(delta < 0 ? node.left : node.right,q,d,answer);
        if (delta*delta < d) nearest(delta < 0 ? node.right : node.left,q,d,answer);
    }
public:
    explicit Tree(std::vector<Point> points) {
        nodes_.reserve(points.size()); root_ = build(points,0,static_cast<int>(points.size()),0);
    }
    double nearest(const Point& q, Point* answer = nullptr) const {
        double distance = std::numeric_limits<double>::infinity(); Point found;
        nearest(root_,q,distance,found);
        if (answer) *answer = found;
        return distance;
    }
    std::size_t size() const { return nodes_.size(); }
};

inline std::vector<Point> voxelize(const std::vector<Point>& input, double size) {
    struct Key {
        long long x,y,z;
        bool operator==(const Key& k) const { return x==k.x && y==k.y && z==k.z; }
    };
    struct Hash {
        std::size_t operator()(const Key& k) const {
            auto a = std::hash<long long>{}(k.x), b = std::hash<long long>{}(k.y), c = std::hash<long long>{}(k.z);
            return a ^ (b+0x9e3779b9+(a<<6)+(a>>2)) ^ (c*0x85ebca6b);
        }
    };
    struct Cell { Point sum; int count = 0; };
    std::unordered_map<Key,Cell,Hash> cells;
    for (const auto& p : input) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            std::max({std::abs(p.x),std::abs(p.y),std::abs(p.z)}) > 1e6) continue;
        auto& cell = cells[{static_cast<long long>(std::floor(p.x/size)),
            static_cast<long long>(std::floor(p.y/size)),static_cast<long long>(std::floor(p.z/size))}];
        cell.sum.x += p.x; cell.sum.y += p.y; cell.sum.z += p.z; ++cell.count;
    }
    std::vector<Point> out; out.reserve(cells.size());
    for (const auto& pair : cells) {
        const auto& c = pair.second; out.push_back({c.sum.x/c.count,c.sum.y/c.count,c.sum.z/c.count});
    }
    std::sort(out.begin(),out.end(),[](const Point& a,const Point& b) {
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        return a.z < b.z;
    });
    return out;
}
inline std::vector<Point> sample(const std::vector<Point>& cloud, int count) {
    if (cloud.size() <= static_cast<std::size_t>(count)) return cloud;
    std::vector<Point> out; out.reserve(count);
    for (int i=0; i<count; ++i) out.push_back(cloud[static_cast<std::size_t>(i)*cloud.size()/count]);
    return out;
}

class Matcher {
    Options options_;
    Tree map_;
    static std::vector<Point> prepare_map(const std::vector<Point>& map, const Options& o) {
        o.validate(); std::vector<Point> cropped;
        const double bound = o.radius+o.scan_range+o.icp_distance;
        for (const auto& p : map) if (squared_distance(p,o.center) <= bound*bound) cropped.push_back(p);
        return voxelize(cropped,o.voxel_size);
    }
    Pose constrain(Pose pose) const {
        double dx=pose.x-options_.center.x, dy=pose.y-options_.center.y;
        double dz=std::clamp(pose.z-options_.center.z,-options_.z_range,options_.z_range);
        dz=std::clamp(dz,-options_.radius,options_.radius);
        double horizontal=std::hypot(dx,dy), limit=std::sqrt(std::max(0.0,options_.radius*options_.radius-dz*dz));
        if (horizontal > limit) { dx*=limit/horizontal; dy*=limit/horizontal; }
        return {options_.center.x+dx,options_.center.y+dy,options_.center.z+dz,angle(pose.yaw)};
    }
    bool distinct(const Pose& a,const Pose& b) const {
        return squared_distance({a.x,a.y,a.z},{b.x,b.y,b.z}) >= options_.distinct_distance*options_.distinct_distance ||
            std::abs(angle(a.yaw-b.yaw)) >= options_.distinct_yaw_deg*pi/180;
    }
    Result score(const std::vector<Point>& cloud, const Pose& pose, double gate) const {
        Result r; r.pose=pose; r.points=static_cast<int>(cloud.size());
        double sum=0,total=0, threshold=gate*gate;
        for (const auto& p : cloud) {
            double distance=map_.nearest(transform(p,pose));
            total+=std::min(distance,threshold);
            if (distance <= threshold) { sum+=distance; ++r.inliers; }
        }
        if (r.points) { r.overlap=double(r.inliers)/r.points; r.cost=std::sqrt(total/r.points); }
        if (r.inliers) r.rmse=std::sqrt(sum/r.inliers);
        r.accepted = r.points >= options_.min_points && r.inliers >= options_.min_points &&
            r.overlap >= options_.min_overlap && r.rmse <= options_.max_rmse && within_bounds(pose);
        r.reason=r.accepted ? "quality_passed" : "poor_match";
        return r;
    }
    Result refine(const std::vector<Point>& cloud, Pose pose, const std::function<bool()>& stop) const {
        struct Pair { Point source,target; double distance; };
        bool converged=false;
        for (int iteration=0; iteration<options_.iterations; ++iteration) {
            if (stop()) break;
            std::vector<Pair> pairs; pairs.reserve(cloud.size());
            for (const auto& p : cloud) {
                Point target; double d=map_.nearest(transform(p,pose),&target);
                if (d < options_.icp_distance*options_.icp_distance) pairs.push_back({p,target,d});
            }
            if (pairs.size() < static_cast<std::size_t>(options_.min_points)) break;
            std::sort(pairs.begin(),pairs.end(),[](const Pair& a,const Pair& b) { return a.distance<b.distance; });
            const std::size_t keep=std::max<std::size_t>(options_.min_points,pairs.size()*9/10);
            Point a{},b{};
            for (std::size_t i=0;i<keep;++i) {
                a.x+=pairs[i].source.x; a.y+=pairs[i].source.y; a.z+=pairs[i].source.z;
                b.x+=pairs[i].target.x; b.y+=pairs[i].target.y; b.z+=pairs[i].target.z;
            }
            a={a.x/keep,a.y/keep,a.z/keep}; b={b.x/keep,b.y/keep,b.z/keep};
            double cosine=0,sine=0;
            for (std::size_t i=0;i<keep;++i) {
                double sx=pairs[i].source.x-a.x, sy=pairs[i].source.y-a.y;
                double tx=pairs[i].target.x-b.x, ty=pairs[i].target.y-b.y;
                cosine+=sx*tx+sy*ty; sine+=sx*ty-sy*tx;
            }
            if (std::hypot(cosine,sine) < 1e-9) break;
            double yaw=std::atan2(sine,cosine), c=std::cos(yaw), s=std::sin(yaw);
            Pose next=constrain({b.x-c*a.x+s*a.y,b.y-s*a.x-c*a.y,b.z-a.z,yaw});
            double delta=squared_distance({next.x,next.y,next.z},{pose.x,pose.y,pose.z});
            double turn=std::abs(angle(next.yaw-pose.yaw)); pose=next;
            if (delta < 1e-6 && turn < 1e-4) { converged=true; break; }
        }
        auto r=score(cloud,pose,options_.inlier_distance); r.converged=converged;
        r.accepted=r.accepted && converged;
        if (!converged) r.reason="not_converged";
        return r;
    }
public:
    Matcher(const std::vector<Point>& map,const Options& o): options_(o),map_(prepare_map(map,o)) {
        if (map_.size() < static_cast<std::size_t>(o.min_points))
            throw std::invalid_argument("Reference map has too few points near the relocalization search region");
    }
    bool within_bounds(const Pose& p) const {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(p.yaw) &&
            squared_distance({p.x,p.y,p.z},options_.center) <= options_.radius*options_.radius+1e-8 &&
            std::abs(p.z-options_.center.z) <= options_.z_range+1e-8;
    }
    std::vector<Point> prepare_scan(const std::vector<Point>& input) const {
        std::vector<Point> ranged;
        for (const auto& p : input) if (p.x*p.x+p.y*p.y+p.z*p.z <= options_.scan_range*options_.scan_range) ranged.push_back(p);
        return sample(voxelize(ranged,options_.voxel_size),options_.max_points);
    }
    Result evaluate(const std::vector<Point>& input,const Pose& pose) const {
        return score(prepare_scan(input),pose,options_.inlier_distance);
    }
    Result search(const std::vector<Point>& input, const std::function<bool()>& cancelled = [] { return false; }) const {
        auto start=std::chrono::steady_clock::now();
        auto elapsed=[&] { return std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count(); };
        auto stop=[&] { return cancelled() || elapsed() > options_.max_seconds; };
        auto finish=[&](Result r) {
            r.seconds=elapsed();
            if (stop()) { r.accepted=false; r.reason=cancelled() ? "cancelled" : "timeout"; }
            return r;
        };
        Result empty;
        auto cloud=prepare_scan(input);
        if (cloud.size() < static_cast<std::size_t>(options_.min_points)) return finish(empty);
        auto coarse=sample(cloud,options_.coarse_points);
        std::vector<Result> seeds;
        int xy=static_cast<int>(std::ceil(options_.radius/options_.xy_step));
        int heights=static_cast<int>(std::ceil(std::min(options_.radius,options_.z_range)/options_.z_step));
        int angles=static_cast<int>(std::ceil(360/options_.yaw_step_deg));
        for (int ix=-xy;ix<=xy;++ix) for (int iy=-xy;iy<=xy;++iy) for (int iz=-heights;iz<=heights;++iz) {
            Pose seed{options_.center.x+ix*options_.xy_step,options_.center.y+iy*options_.xy_step,
                options_.center.z+std::clamp(iz*options_.z_step,-options_.z_range,options_.z_range),0};
            if (!within_bounds(seed)) continue;
            for (int a=0;a<angles;++a) {
                if (stop()) return finish(empty);
                seed.yaw=angle(a*2*pi/angles);
                seeds.push_back(score(coarse,seed,options_.coarse_distance));
            }
        }
        std::sort(seeds.begin(),seeds.end(),[](const Result& a,const Result& b) { return a.cost<b.cost; });
        std::vector<Pose> selected;
        for (const auto& seed : seeds) {
            bool different=true;
            for (const auto& p : selected) if (!distinct(seed.pose,p)) { different=false; break; }
            if (different) selected.push_back(seed.pose);
            if (selected.size() >= static_cast<std::size_t>(options_.candidates)) break;
        }
        std::vector<Result> refined;
        for (const auto& p : selected) {
            if (stop()) return finish(empty);
            refined.push_back(refine(cloud,p,stop));
        }
        std::sort(refined.begin(),refined.end(),[](const Result& a,const Result& b) { return a.cost<b.cost; });
        auto best=std::find_if(refined.begin(),refined.end(),[](const Result& r) { return r.accepted; });
        if (best==refined.end()) return finish(refined.empty() ? empty : refined.front());
        Result answer=*best; answer.reason="matched";
        for (const auto& other : refined) {
            if (other.converged && other.overlap >= options_.min_overlap-0.05 &&
                other.rmse <= options_.max_rmse+0.03 && distinct(answer.pose,other.pose) &&
                other.cost-answer.cost < options_.ambiguity_margin) {
                answer.accepted=false; answer.reason="ambiguous"; break;
            }
        }
        return finish(answer);
    }
};
} // namespace fastlio_relocalization
