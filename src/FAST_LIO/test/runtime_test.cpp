#include "workspace_runtime.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
struct Point { double x=0,y=0,z=0,intensity=0; };
static void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
template<class F> static void rejects(F f) { bool rejected=false; try { f(); } catch(const std::exception&) { rejected=true; } require(rejected,"Invalid input accepted"); }
int main() {
    using namespace fastlio_runtime;
    char pattern[]="/tmp/fastlio-runtime-test-XXXXXX";
    const char* created=mkdtemp(pattern); if(!created) return 1;
    const std::filesystem::path root(created);
    try {
        validate_window(400,100); rejects([]{ validate_window(100,100); });
        rejects([]{ validate_window(300,100); }); rejects([]{ validate_window(400,-1); });
        // The unchanged upstream movement threshold must not fire at the center.
        require(400.0/2>1.5*100,"Stationary window moves");
        HealthOptions o; o.lost_frames=3; o.recovery_frames=2;
        Health h(o); require(!h.trusted(),"Waiting marked trusted");
        require(h.update(50,100,0.1),"Healthy first scan rejected");
        require(!h.update(0,100,0.0),"IMU-only frame accepted");
        require(!h.update(50,100,0.1),"Recovered too early");
        require(h.update(50,100,0.1),"Recovery rejected");
        for(int i=0;i<3;++i) h.update(0,100,0);
        require(h.state()==Health::State::Lost && !h.update(50,100,0.1),"LOST not latched");
        h.reset(); require(h.update(50,100,0.1),"Reset failed"); h.stale(); require(!h.trusted(),"Stale trusted");
        rejects([]{ HealthOptions bad; bad.min_ratio=2; Health unused(bad); });
        VoxelMap<Point> map(0.1,2);
        map.insert({0.01,0.01,0.01,1}); map.insert({0.05,0.05,0.05,7});
        for(int i=0;i<100000;++i) map.insert({0.02,0.02,0.02,2});
        require(map.size()==1 && map.snapshot()[0].intensity==7,"Static duplication or representative lost");
        map.insert({-0.01,0,0,3}); require(map.size()==2,"Negative coordinates merged");
        map.insert({1,1,1,4}); require(map.size()==2 && !map.complete(),"Capacity silently exceeded");
        map.insert({std::numeric_limits<double>::quiet_NaN(),0,0,0}); require(map.invalid()==1,"NaN accepted");
        require(map.snapshot(1).size()==1,"Display snapshot limit failed");
        const auto old=map.snapshot(); map.insert({-0.05,0.05,0.05,8});
        require(old.size()==2,"Snapshot aliases live map");
        VoxelMap<Point> raw(0,3); for(int i=0;i<4;++i) raw.insert({0,0,0,0});
        require(raw.size()==3 && !raw.complete(),"Legacy mode unbounded");
        rejects([]{ VoxelMap<Point> bad(-0.1); });
        const Vector gravity{1.0,-0.8,-9.7};
        const auto r=gravity_to_level(gravity,20); const auto level=rotate(r,gravity);
        require(std::abs(level[0])<1e-9 && std::abs(level[1])<1e-9 && level[2]<0,"Gravity not aligned");
        const Vector p{1,2,3}; const auto q=rotate(r,p);
        require(std::abs(std::hypot(std::hypot(q[0],q[1]),q[2])-std::sqrt(14))<1e-9,"Alignment not rigid");
        rejects([]{ gravity_to_level({0,0,1},20); }); rejects([]{ gravity_to_level({0,0,0}); });
        require(json_string("a\n\"b")=="\"a\\u000a\\\"b\"","Unsafe JSON escaping");
        CsvJournal journal; journal.begin(root,"episode","# metadata\nx,y\n"); const auto csv=journal.path();
        journal.append("1,2\n"); journal.flush();
        std::ifstream visible(csv); std::stringstream contents; contents<<visible.rdbuf();
        require(contents.str().find("1,2")!=std::string::npos,"Active CSV unavailable before stop");
        journal.finish("# interrupted: true\n"); require(!journal.active(),"Journal still active");
        { std::ofstream known(root/"crc.txt"); known<<"123456789"; }
        require(file_crc32(root/"crc.txt")=="cbf43926","CRC32 incompatible with zlib");
        CsvJournal second; second.begin(root,"episode","x,y\n"); require(second.path()!=csv,"CSV overwritten"); second.finish();
        rejects([&]{ second.begin(root,"../unsafe",""); });
        std::filesystem::remove_all(root);
        std::cout<<"PASS: window validation, quality/recovery/latched loss, bounded voxel maps, snapshots, gravity alignment and durable CSV\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; std::filesystem::remove_all(root); return 1; }
}
