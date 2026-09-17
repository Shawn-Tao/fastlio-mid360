#pragma once
// ROS/PCL-independent policies. The ROS executor owns these objects; workers
// receive immutable snapshots, never references to the live map or recorder.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <unistd.h>

namespace fastlio_runtime {
inline std::string file_crc32(const std::filesystem::path& path) {
    static const auto table=[] {
        std::array<std::uint32_t,256> values{};
        for(std::uint32_t i=0;i<256;++i) {
            std::uint32_t c=i; for(int bit=0;bit<8;++bit) c=(c&1)?0xedb88320U^(c>>1):(c>>1);
            values[i]=c;
        }
        return values;
    }();
    std::ifstream input(path,std::ios::binary); if(!input) throw std::runtime_error("Cannot read PCD for checksum");
    std::array<char,65536> buffer{}; std::uint32_t crc=0xffffffffU;
    while(input) {
        input.read(buffer.data(),buffer.size());
        for(std::streamsize i=0;i<input.gcount();++i) crc=table[(crc^static_cast<unsigned char>(buffer[i]))&255]^(crc>>8);
    }
    if(!input.eof()) throw std::runtime_error("PCD checksum read failed");
    std::ostringstream out; out<<std::hex<<std::setw(8)<<std::setfill('0')<<(crc^0xffffffffU); return out.str();
}
inline void validate_window(double side, double range) {
    if (!std::isfinite(side) || !std::isfinite(range) || range <= 0 || side <= 3*range)
        throw std::invalid_argument("cube_side_length must be finite and > 3 * mapping.det_range (both positive)");
}
inline std::string json_string(const std::string& input) {
    std::string out="\"";
    for (unsigned char c : input) {
        if (c=='"' || c=='\\') { out+='\\'; out+=char(c); }
        else if (c<32) { char escaped[7]; std::snprintf(escaped,sizeof(escaped),"\\u%04x",c); out+=escaped; }
        else out+=char(c);
    }
    return out+'"';
}

struct HealthOptions {
    int min_points=20, lost_frames=10, recovery_frames=3;
    double min_ratio=0.05, max_residual=0.20, stale_seconds=1.0;
    void validate() const {
        if (min_points<5 || lost_frames<1 || recovery_frames<1 ||
            !std::isfinite(min_ratio) || min_ratio<=0 || min_ratio>1 ||
            !std::isfinite(max_residual) || max_residual<=0 ||
            !std::isfinite(stale_seconds) || stale_seconds<=0)
            throw std::invalid_argument("Invalid tracking quality/time limits");
    }
};
class Health {
public:
    enum class State { Waiting, Tracking, Degraded, Lost };
private:
    HealthOptions options_;
    State state_=State::Waiting;
    int bad_=0, good_=0;
public:
    explicit Health(HealthOptions options={}) : options_(options) { options_.validate(); }
    void reset() { state_=State::Waiting; bad_=good_=0; }
    State state() const { return state_; }
    const char* name() const {
        switch(state_) {
            case State::Waiting: return "waiting";
            case State::Tracking: return "tracking";
            case State::Degraded: return "degraded";
            case State::Lost: return "lost";
        }
        return "lost";
    }
    bool trusted() const { return state_==State::Tracking; }
    // LOST is latched: never silently jump/recover in the middle of a recording.
    bool update(int matched,int total,double residual,bool finite=true) {
        if (state_==State::Lost) return false;
        const bool valid=finite && total>0 && matched>=options_.min_points &&
            double(matched)/total>=options_.min_ratio && std::isfinite(residual) && residual<=options_.max_residual;
        if (!valid) {
            good_=0;
            state_=++bad_>=options_.lost_frames ? State::Lost : State::Degraded;
            return false;
        }
        bad_=0;
        if (state_==State::Waiting || state_==State::Tracking || ++good_>=options_.recovery_frames) {
            state_=State::Tracking; good_=0;
        }
        return trusted();
    }
    void stale() { state_=State::Lost; good_=bad_=0; }
};

template<class Point> class VoxelMap {
    struct Key {
        std::int64_t x,y,z;
        bool operator==(const Key& b) const { return x==b.x && y==b.y && z==b.z; }
    };
    struct Hash {
        std::size_t operator()(const Key& k) const {
            std::size_t h=std::hash<std::int64_t>{}(k.x);
            h^=std::hash<std::int64_t>{}(k.y)+0x9e3779b9+(h<<6)+(h>>2);
            h^=std::hash<std::int64_t>{}(k.z)+0x9e3779b9+(h<<6)+(h>>2);
            return h;
        }
    };
    double size_;
    std::size_t limit_;
    std::unordered_map<Key,Point,Hash> cells_;
    std::vector<Point> raw_;
    std::uint64_t rejected_=0, invalid_=0, input_=0;
    double distance(const Point& p,const Key& k) const {
        const double x=p.x-(k.x+0.5)*size_, y=p.y-(k.y+0.5)*size_, z=p.z-(k.z+0.5)*size_;
        return x*x+y*y+z*z;
    }
public:
    // size==0 explicitly selects legacy raw accumulation, still capacity-limited.
    VoxelMap(double size=0.1,std::size_t limit=2000000):size_(size),limit_(limit) {
        if (!std::isfinite(size) || size<0 || (size>0 && size<1e-4))
            throw std::invalid_argument("pcd_save.voxel_size must be 0 (disabled) or >= 0.0001 m");
    }
    bool insert(const Point& p) {
        ++input_;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            std::max({std::abs(double(p.x)),std::abs(double(p.y)),std::abs(double(p.z))})>1e6) {
            ++invalid_; return false;
        }
        if (size_==0) {
            if (limit_ && raw_.size()>=limit_) { ++rejected_; return false; }
            raw_.push_back(p); return true;
        }
        const Key k{std::int64_t(std::floor(p.x/size_)),std::int64_t(std::floor(p.y/size_)),std::int64_t(std::floor(p.z/size_))};
        auto found=cells_.find(k);
        if (found!=cells_.end()) {
            if (distance(p,k)<distance(found->second,k)) { found->second=p; return true; }
            return false;
        }
        if (limit_ && cells_.size()>=limit_) { ++rejected_; return false; }
        cells_.emplace(k,p); return true;
    }
    std::size_t size() const { return size_==0 ? raw_.size() : cells_.size(); }
    bool complete() const { return rejected_==0; }
    std::uint64_t rejected() const { return rejected_; }
    std::uint64_t invalid() const { return invalid_; }
    std::uint64_t input() const { return input_; }
    template<class Container> void snapshot_into(Container& out,std::size_t display_limit=0) const {
        const std::size_t stride=display_limit && size()>display_limit ? (size()+display_limit-1)/display_limit : 1;
        out.clear(); out.reserve((size()+stride-1)/stride);
        std::size_t i=0;
        if (size_==0) { for (const auto& p:raw_) if (i++%stride==0) out.push_back(p); }
        else { for (const auto& cell:cells_) if (i++%stride==0) out.push_back(cell.second); }
    }
    std::vector<Point> snapshot(std::size_t display_limit=0) const {
        std::vector<Point> out; snapshot_into(out,display_limit);
        return out;
    }
};

using Vector=std::array<double,3>;
using Rotation=std::array<double,9>; // row-major
inline Vector rotate(const Rotation& r,const Vector& p) {
    return {r[0]*p[0]+r[1]*p[1]+r[2]*p[2],r[3]*p[0]+r[4]*p[1]+r[5]*p[2],r[6]*p[0]+r[7]*p[1]+r[8]*p[2]};
}
inline Rotation gravity_to_level(Vector gravity,double max_tilt_deg=89) {
    const double norm=std::hypot(std::hypot(gravity[0],gravity[1]),gravity[2]);
    if (!std::isfinite(norm) || norm<1e-6 || !std::isfinite(max_tilt_deg) || max_tilt_deg<=0 || max_tilt_deg>=90)
        throw std::invalid_argument("Invalid gravity vector / upright tilt limit");
    const Vector up{-gravity[0]/norm,-gravity[1]/norm,-gravity[2]/norm};
    const double cosine=std::clamp(up[2],-1.0,1.0);
    if (std::acos(cosine)*180/3.14159265358979323846>max_tilt_deg)
        throw std::invalid_argument("Startup tilt exceeds localization.relocalization.max_tilt_deg");
    const double x=up[1],y=-up[0],d=1/(1+cosine);
    return {1-y*y*d,x*y*d,y, x*y*d,1-x*x*d,-x, -y,x,1-(x*x+y*y)*d};
}

class CsvJournal {
    FILE* file_=nullptr;
    std::filesystem::path path_;
public:
    ~CsvJournal() { if(file_) std::fclose(file_); }
    CsvJournal()=default;
    CsvJournal(const CsvJournal&)=delete;
    CsvJournal& operator=(const CsvJournal&)=delete;
    const std::filesystem::path& path() const { return path_; }
    bool active() const { return file_!=nullptr; }
    void begin(const std::filesystem::path& directory,const std::string& prefix,const std::string& header) {
        if(file_) throw std::runtime_error("CSV recording already active");
        if(prefix.empty() || prefix.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)
            throw std::invalid_argument("Invalid CSV filename prefix");
        std::filesystem::create_directories(directory);
        std::string pattern=(directory/(prefix+"_XXXXXX.csv")).string();
        std::vector<char> writable(pattern.begin(),pattern.end()); writable.push_back('\0');
        const int fd=mkstemps(writable.data(),4);
        if(fd<0) throw std::runtime_error("Cannot create CSV recording");
        path_=writable.data(); file_=fdopen(fd,"w");
        if(!file_) { close(fd); throw std::runtime_error("Cannot open CSV recording"); }
        append(header); flush();
        const int dir_fd=::open(directory.c_str(),O_RDONLY | O_DIRECTORY);
        if(dir_fd<0) throw std::runtime_error("Cannot open CSV directory for durable sync");
        const int synced=fsync(dir_fd); close(dir_fd);
        if(synced!=0) throw std::runtime_error("CSV directory durable sync failed");
    }
    void append(const std::string& line) {
        if(!file_ || std::fwrite(line.data(),1,line.size(),file_)!=line.size())
            throw std::runtime_error("CSV write failed");
    }
    void flush() {
        if(file_ && (std::fflush(file_)!=0 || fdatasync(fileno(file_))!=0))
            throw std::runtime_error("CSV durable flush failed");
    }
    void finish(const std::string& footer="") {
        if(!file_) return;
        append(footer); flush();
        FILE* closing=file_; file_=nullptr;
        if(std::fclose(closing)!=0) throw std::runtime_error("CSV close failed");
    }
};
} // namespace fastlio_runtime
