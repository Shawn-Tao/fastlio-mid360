#include "bounded_relocalization.hpp"
#include "startup_relocalization.hpp"
#include <iostream>
#include <random>
#include <thread>

using namespace fastlio_relocalization;
static void require(bool value,const char* message) {
    if (!value) throw std::runtime_error(message);
}
static std::vector<Point> scene() {
    std::vector<Point> map;
    // An asymmetric room: floor, walls, and two differently shaped objects.
    for (double x=-4;x<5;x+=0.17) for (double y=-3;y<4;y+=0.17) {
        map.push_back({x,y,-0.7});
        if (x>1.4 && x<2.4 && y>0.3 && y<1.5) map.push_back({x,y,0.8});
    }
    for (double z=-0.7;z<1.8;z+=0.17) {
        for (double x=-4;x<5;x+=0.17) { map.push_back({x,-3,z}); map.push_back({x,4,z}); }
        for (double y=-3;y<4;y+=0.17) { map.push_back({-4,y,z}); map.push_back({5,y,z}); }
        for (double y=-2;y<0;y+=0.17) map.push_back({-1.3,y,z});
    }
    return map;
}
static std::vector<Point> observation(const std::vector<Point>& map,const Pose& pose,bool noise=true) {
    std::vector<Point> scan;
    std::mt19937 rng(42); std::normal_distribution<double> perturb(0,0.008);
    const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
    for (std::size_t i=0;i<map.size();++i) {
        if (i%7==0) continue; // partial observation
        const auto& p=map[i]; double dx=p.x-pose.x,dy=p.y-pose.y;
        Point q{c*dx+s*dy,-s*dx+c*dy,p.z-pose.z};
        if (noise) { q.x+=perturb(rng); q.y+=perturb(rng); q.z+=perturb(rng); }
        scan.push_back(q);
    }
    return scan;
}
static void print(const Result& r) {
    std::cout<<r.reason<<" pose="<<r.pose.x<<","<<r.pose.y<<","<<r.pose.z<<","<<r.pose.yaw
        <<" overlap="<<r.overlap<<" rmse="<<r.rmse<<" seconds="<<r.seconds<<"\n";
}
int main() {
    try {
        Options o; o.max_seconds=30;
        const auto map=scene(); Matcher matcher(map,o);
        const Pose actual{2.35,-1.15,0.22,2.4};
        auto scan=observation(map,actual);
        auto r=matcher.search(scan); print(r);
        require(r.accepted,"Bounded search rejected asymmetric scene");
        require(squared_distance({r.pose.x,r.pose.y,r.pose.z},{actual.x,actual.y,actual.z})<0.01,"Incorrect recovered position");
        require(std::abs(angle(r.pose.yaw-actual.yaw))<0.02,"Incorrect recovered heading");
        require(matcher.within_bounds(r.pose),"Recovered pose escaped search radius");
        require(matcher.evaluate(observation(map,actual),r.pose).accepted,"Independent scan confirmation failed");
        auto reversed=matcher.search(observation(map,{0.2,0.1,0.0,-pi+0.02})); print(reversed);
        require(reversed.accepted && std::abs(angle(reversed.pose.yaw+pi-0.02))<0.03,"Full heading search failed at pi boundary");
        // Different scene and poses outside the specified search volume must fail.
        std::vector<Point> unrelated;
        for (int i=0;i<500;++i) unrelated.push_back({8.0+i%20*0.08,8.0+i/20*0.08,6.0});
        require(!matcher.search(unrelated).accepted,"Unrelated scene accepted");
        require(!matcher.evaluate(scan,{4,0,0,0}).accepted,"Out-of-radius pose accepted");
        require(!matcher.evaluate(scan,{0,0,0.8,0}).accepted,"Out-of-height pose accepted");
        require(!matcher.search({}).accepted,"Empty scan accepted");
        auto cancelled=matcher.search(scan,[] { return true; });
        require(!cancelled.accepted && cancelled.reason=="cancelled","Cancellation failed");
        Options deadline=o; deadline.max_seconds=1e-9;
        auto timeout=Matcher(map,deadline).search(scan);
        require(!timeout.accepted && timeout.reason=="timeout","Incomplete timed-out search accepted");
        std::vector<Point> shape;
        std::mt19937 rng(4); std::uniform_real_distribution<double> random(-0.3,0.3);
        for (int i=0;i<700;++i) shape.push_back({random(rng),random(rng),random(rng)});
        std::vector<Point> twins;
        for (const auto& p : shape) { twins.push_back({p.x-1.5,p.y,p.z}); twins.push_back({p.x+1.5,p.y,p.z}); }
        Options symmetric=o; symmetric.voxel_size=0.06; symmetric.min_points=80;
        symmetric.candidates=48; symmetric.xy_step=0.5;
        auto ambiguity=Matcher(twins,symmetric).search(shape); print(ambiguity);
        require(!ambiguity.accepted && ambiguity.reason=="ambiguous","Duplicate places not rejected as ambiguous");
        bool invalid=false;
        try { Options bad=o; bad.radius=-1; Matcher unused(map,bad); } catch(const std::invalid_argument&) { invalid=true; }
        require(invalid,"Invalid radius accepted");
        invalid=false;
        try { Options bad=o; bad.xy_step=1e-5; Matcher unused(map,bad); } catch(const std::invalid_argument&) { invalid=true; }
        require(invalid,"Unbounded seed count accepted");
        invalid=false;
        try { Matcher unused({},o); } catch(const std::invalid_argument&) { invalid=true; }
        require(invalid,"Empty map accepted");
        scan.push_back({std::numeric_limits<double>::quiet_NaN(),0,0});
        require(matcher.evaluate(scan,r.pose).accepted,"Nonfinite scan point broke confirmation");
        auto engine=std::make_shared<Matcher>(map,o);
        Startup startup(engine,2,2);
        require(!startup.ready() && startup.phase()==Startup::Phase::WaitingImu,"Startup did not gate before IMU init");
        require(!startup.update(scan,true,false) && !startup.ready(),"Uninitialized IMU bypassed gate");
        startup.update(scan,true,true);
        startup.update(scan,false,true); // motion discards the first accumulated scan
        startup.update(scan,true,true);
        require(startup.phase()==Startup::Phase::Collecting,"Moving collection was not reset");
        startup.update(scan,true,true);
        require(startup.phase()==Startup::Phase::Matching && !startup.ready(),"Search snapshot bypassed quality gate");
        require(!startup.reset(),"Busy worker was reset unsafely");
        auto end=std::chrono::steady_clock::now()+std::chrono::seconds(40);
        while(startup.phase()==Startup::Phase::Matching && std::chrono::steady_clock::now()<end) {
            startup.update(scan,true,true); std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        require(startup.phase()==Startup::Phase::Confirming && !startup.ready(),"Tentative match bypassed fresh scan confirmation");
        require(!startup.update(scan,true,true) && !startup.ready(),"One confirmation frame bypassed configured count");
        require(startup.update(scan,true,true) && startup.ready(),"Fresh stationary confirmation never initialized tracking");
        require(!startup.update(scan,true,true),"READY transition occurred twice");
        require(startup.reset() && !startup.ready(),"Explicit retry did not invalidate old localization");
        startup.update(scan,true,true); startup.update(scan,true,true);
        end=std::chrono::steady_clock::now()+std::chrono::seconds(40);
        while(startup.phase()==Startup::Phase::Matching && std::chrono::steady_clock::now()<end) {
            startup.update(scan,false,true); std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        require(startup.phase()==Startup::Phase::Failed && !startup.ready() && startup.detail()=="motion_during_search",
            "Moving robot initialized from stale search snapshot");
        require(startup.reset(),"Motion failure could not be retried");
        startup.update(scan,true,true); startup.update(scan,true,true);
        end=std::chrono::steady_clock::now()+std::chrono::seconds(40);
        while(startup.phase()==Startup::Phase::Matching && std::chrono::steady_clock::now()<end) {
            startup.update(scan,true,true); std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        require(startup.phase()==Startup::Phase::Confirming,"Retry failed to produce tentative match");
        require(!startup.update(unrelated,true,true) && !startup.ready() && startup.phase()==Startup::Phase::Failed,
            "Bad independent scan bypassed confirmation gate");
        std::cout<<"PASS: position/360-degree yaw recovery, holdout confirmation, bounds, mismatch, ambiguity, cancellation, timeout and parameter validation\n";
        std::cout<<"PASS: IMU/collection/search/fresh-confirmation gate, motion rejection, busy-worker safety, retry and one-shot READY transition\n";
        return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<"\n"; return 1; }
}
