#include "static_map.hpp"
#include <iostream>
#include <random>

struct Point { double x=0,y=0,z=0,intensity=0; };
using Map=fastlio_runtime::StaticVoxelMap<Point>;
using Points=std::vector<Point>;
static const fastlio_runtime::Vector origin{0.05,0.05,0.05};
static void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
template<class F> static void rejects(F f) {
    bool rejected=false; try { f(); } catch(const std::invalid_argument&) { rejected=true; }
    require(rejected,"Invalid configuration accepted");
}
static fastlio_runtime::StaticMapOptions quick() {
    fastlio_runtime::StaticMapOptions o;
    o.min_observations=3; o.confirmation_seconds=0.5; o.observation_interval=0.25;
    o.clear_observations=3; o.clear_seconds=0.5; o.clear_vote_ttl=2;
    o.candidate_ttl=3; o.max_rays=32;
    return o;
}
static void seed(Map& map,const Points& points) {
    for(double stamp:{0.0,0.25,0.5}) require(map.insert_frame(points,origin,stamp),"Seed failed");
}
static Points snapshot(const Map& map) { Points out; map.snapshot_into(out); return out; }

int main(int argc,char** argv) {
    try {
        auto o=quick();
        const Points person{{3.05,0.05,0.05,7}},wall{{6.05,0.05,0.05,9}};
        {
            Map map(.1,100,o); Points duplicates(10000,person[0]);
            map.insert_frame(duplicates,origin,0);
            require(map.size()==0 && map.candidates()==1,"One scan/duplicate points confirmed occupancy");
            map.insert_frame(duplicates,origin,.1);
            require(map.size()==0,"Observation interval ignored");
            map.insert_frame(duplicates,origin,.25);
            require(map.size()==0,"Confirmation span ignored");
            map.insert_frame(person,origin,.5);
            require(map.size()==1 && map.candidates()==0 && snapshot(map)[0].intensity==7,"Static point/attributes not promoted");
        }
        {
            Map map(.1,100,o);
            for(int i=0;i<20;++i) map.insert_frame(Points{{1.05+i*.2,.05,.05,0}},origin,i*.25,false);
            require(map.size()==0,"Moving transient points persisted");
            map.insert_frame(Points{},origin,10);
            require(map.candidates()==0 && map.complete(),"Candidate TTL or completeness incorrect");
        }
        {
            Map map(.1,100,o); seed(map,person);
            const auto immutable=snapshot(map);
            map.insert_frame(wall,origin,.75);
            require(map.size()==1,"One free ray deleted occupancy");
            map.insert_frame(wall,origin,1);
            require(map.size()==1,"Cleared before repeated evidence/span");
            map.insert_frame(wall,origin,1.25);
            require(map.cleared()==1 && map.size()==1 && snapshot(map)[0].intensity==9,"Standing then departing person not cleared, or wall removed");
            require(immutable[0].intensity==7,"Snapshot changed during cleanup");
            for(int i=0;i<100;++i) map.insert_frame(wall,origin,1.5+i*.25);
            require(map.size()==1 && map.cleared()==1,"Wall endpoints classified as free space");
            map.insert_frame(person,origin,30);
            require(map.size()==1 && map.candidates()==1,"Cleared point resurrected without fresh confirmation");
        }
        {
            Map map(.1,100,o); seed(map,person);
            for(int i=0;i<20;++i) map.insert_frame(Points{{2.05,.05,.05,0},{6.05,.05,.05,0}},origin,.75+i*.25);
            require(map.cleared()==0 && map.size()==3,"A nearer current return failed to occlude the old surface");
        }
        {
            Map map(.1,100,o); seed(map,person);
            for(int i=0;i<10;++i) map.insert_frame(wall,origin,.75+i*.25,false);
            require(map.cleared()==0,"Uncertain/fast-motion frame cleared the map");
            map.insert_frame(Points{},origin,1000);
            require(map.size()==2,"Historical unobserved area aged out");
        }
        {
            Map map(.1,100,o); seed(map,person);
            map.insert_frame(wall,origin,.75); map.insert_frame(wall,origin,1);
            map.insert_frame(person,origin,1.25);
            map.insert_frame(wall,origin,1.5); map.insert_frame(wall,origin,1.75);
            require(map.cleared()==0,"Positive observation did not reset negative evidence");
            map.insert_frame(wall,origin,2);
            require(map.cleared()==1,"Clear votes never recovered after reset");
        }
        {
            Map map(.1,100,o); seed(map,person);
            map.insert_frame(wall,origin,.75); map.insert_frame(wall,origin,1);
            map.insert_frame(wall,origin,10);
            require(map.cleared()==0,"Stale free-space votes reused after visibility gap");
        }
        {
            Map map(.1,100,o); seed(map,person);
            const Points edge{{3.25,.05,.05,0},{6.05,.25,.05,0}};
            for(int i=0;i<10;++i) map.insert_frame(edge,origin,.75+i*.25);
            require(map.cleared()==0,"Endpoint margin or narrow ray clearance ignored");
        }
        {
            Map map(.1,100,o); seed(map,Points{{-3.05,-.05,-.05,1},{3.05,.05,.05,2}});
            require(map.size()==2,"Negative-coordinate voxels merged");
            const fastlio_runtime::Vector negative_origin{-.05,-.05,-.05};
            for(double t:{.75,1.0,1.25}) map.insert_frame(Points{{-6.05,-.05,-.05,0}},negative_origin,t);
            require(map.cleared()==1 && map.size()==2,"Negative direction ray traversal failed");
            Points display; map.snapshot_into(display,1); require(display.size()==1,"Display limit failed");
        }
        {
            Map map(.1,100,o); seed(map,person);
            require(!map.insert_frame(wall,origin,.5),"Duplicate timestamp accepted");
            require(!map.insert_frame(wall,origin,.1),"Timestamp regression accepted");
            require(!map.insert_frame(wall,origin,std::numeric_limits<double>::quiet_NaN()),"NaN timestamp accepted");
            require(!map.insert_frame(wall,{std::numeric_limits<double>::infinity(),0,0},1),"Invalid ray origin accepted");
            map.insert_frame(Points{{std::numeric_limits<double>::quiet_NaN(),0,0,0},{1e7,0,0,0}},origin,1);
            require(map.size()==1 && map.invalid()==2,"Invalid input modified occupancy");
        }
        {
            auto small=o; small.max_candidates=2; Map map(.1,100,small);
            map.insert_frame(Points{{1,1,1,0},{2,2,2,0},{3,3,3,0}},origin,0);
            require(map.candidates()==2 && !map.complete(),"Candidate cap silently dropped new areas");
            map.insert_frame(Points{},origin,10);
            require(map.candidates()==0 && !map.complete(),"Capacity truncation not latched");
        }
        {
            Map map(.1,1,o); seed(map,Points{{1,1,1,0},{2,2,2,0}});
            require(map.size()==1 && !map.complete() && map.rejected()>0,"Confirmed map cap ignored");
            auto small=o; small.max_frame_points=2; Map frame(.1,100,small);
            frame.insert_frame(Points{{1,1,1,0},{2,2,2,0},{3,3,3,0}},origin,0);
            require(frame.candidates()==2 && frame.input()==3 && !frame.complete(),"Frame scratch cap ignored");
        }
        {
            auto small=o; small.max_rays=2; small.max_ray_steps=5; Map map(.1,100,small);
            map.insert_frame(Points{{10,0,0,0},{10,1,0,0},{10,2,0,0}},origin,0);
            const auto status=map.status_json();
            require(status.find("\"last_rays\":2")!=std::string::npos &&
                    status.find("\"last_ray_steps\":10")!=std::string::npos && map.complete(),"Ray workload not bounded or partial visibility marked map truncated");
        }
        {
            auto small=o; small.max_frame_points=1; Map map(.1,100,small); seed(map,person);
            for(int i=0;i<10;++i) map.insert_frame(Points{wall[0],{2.05,.05,.05,0}},origin,.75+i*.25);
            require(map.cleared()==0 && !map.complete(),"Truncated frame used missing endpoints as free space");
            Map converted(.1,100,small); seed(converted,person);
            for(int i=0;i<10;++i) converted.insert_frame(wall,origin,.75+i*.25,true,100);
            require(converted.cleared()==0 && !converted.complete() && converted.input()==1013,
                    "Caller-omitted endpoints not accounted/protected");
        }
        {
            Map map(.1,100,o); seed(map,person);
            Points many_rays;
            for(int i=0;i<20;++i) many_rays.push_back({6.05+i*.1,.05,.05,0});
            map.insert_frame(many_rays,origin,.75);
            require(map.cleared()==0,"Several rays from a single frame counted as independent misses");
        }
        {
            Map map(.1,100,o);
            for(int i=0;i<1000;++i) map.insert_frame(Points{{3.01,.01,.01,1},{3.05,.05,.05,7}},origin,i*.25,false);
            require(map.size()==1 && map.candidates()==0 && snapshot(map)[0].intensity==7,"Repeated hits leaked candidate queue or lost actual representative");
        }
        {
            // Exercise general 3-D/negative directions, not just axis-aligned rays.
            std::mt19937 generator(360);
            std::uniform_real_distribution<double> random(-1,1);
            for(int i=0;i<400;++i) {
                const fastlio_runtime::Vector start{random(generator),random(generator),random(generator)};
                fastlio_runtime::Vector d{random(generator),random(generator),random(generator)};
                const double length=std::hypot(std::hypot(d[0],d[1]),d[2]);
                for(auto& v:d) v/=length;
                const Points obstacle{{start[0]+3*d[0],start[1]+3*d[1],start[2]+3*d[2],1}};
                const Points back{{start[0]+6*d[0],start[1]+6*d[1],start[2]+6*d[2],2}};
                Map map(.1,100,o);
                for(double t:{0.0,.25,.5}) map.insert_frame(obstacle,start,t);
                for(double t:{.75,1.0,1.25}) map.insert_frame(back,start,t);
                require(map.cleared()==1 && map.size()==1,"General 3-D free-ray traversal missed occupancy");
            }
        }
        {
            auto legacy=o; legacy.enabled=false; Map raw(0,3,legacy);
            raw.insert_frame(Points(4,person[0]),origin,0);
            require(raw.size()==3 && raw.candidates()==0 && !raw.complete(),"Disabled filter changed legacy archive semantics");
            Map voxel(.1,100,legacy);
            voxel.insert_frame(Points{{.01,.01,.01,1},{.05,.05,.05,7}},origin,0);
            require(voxel.size()==1 && snapshot(voxel)[0].intensity==7,"Disabled voxel representative changed");
        }
        rejects([&]{ Map bad(0,100,o); }); rejects([&]{ Map bad(.1,0,o); });
        rejects([&]{ auto bad=o; bad.ray_clearance=.051; Map map(.1,100,bad); });
        rejects([&]{ auto bad=o; bad.max_candidates=0; Map map(.1,100,bad); });
        rejects([&]{ auto bad=o; bad.candidate_ttl=.25; Map map(.1,100,bad); });
        rejects([&]{ auto bad=o; bad.clear_vote_ttl=.25; Map map(.1,100,bad); });
        rejects([&]{ auto bad=o; bad.endpoint_margin=100; Map map(.1,100,bad); });
        rejects([&]{ auto bad=o; bad.confirmation_seconds=std::numeric_limits<double>::infinity(); Map map(.1,100,bad); });
        if(argc>1 && (std::string(argv[1])=="--benchmark" || std::string(argv[1])=="--benchmark-large")) {
            Map map(.1,2000000); Points frame; frame.reserve(20000);
            for(int i=0;i<20000;++i) frame.push_back({1.05+(i%200)*.1,1.05+((i/200)%100)*.1,.05,1});
            const int groups=std::string(argv[1])=="--benchmark-large"?25:1;
            double stamp=100;
            for(int group=0;group<groups;++group) {
                if(group) for(auto& p:frame) p.x+=40;
                for(int i=0;i<4;++i) { map.insert_frame(frame,{origin[0]+40*group,origin[1],origin[2]},stamp,false); stamp+=.5; }
            }
            std::vector<double> times;
            for(int i=0;i<300;++i) {
                const auto start=std::chrono::steady_clock::now();
                map.insert_frame(frame,{origin[0]+40*(groups-1),origin[1],origin[2]},stamp+i*.1);
                times.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
            }
            std::sort(times.begin(),times.end());
            require(map.size()==frame.size()*groups && map.candidates()==0 && map.complete(),"Steady static benchmark leaked/duplicated voxels");
            std::cout<<"BENCHMARK synthetic host only: "<<map.size()<<" archived points, 20000 input points x 300 frames; median_ms="<<times[150]
                     <<" p95_ms="<<times[285]<<" max_ms="<<times.back()<<" "<<map.status_json()<<'\n';
        }
        std::cout<<"PASS: archive confirmations, transient/standing people, visibility/reset/occlusion, negative rays, historical retention, bounded caches/work, invalid inputs and legacy mode\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
