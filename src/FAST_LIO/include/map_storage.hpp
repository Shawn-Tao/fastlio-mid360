#pragma once

#include <chrono>
#include <cctype>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>
#include <pcl/io/pcd_io.h>

namespace fastlio_maps {
inline bool valid_name(const std::string& name) {
    if (name.empty()) return false;
    for (unsigned char ch : name) {
        if (!(std::isalnum(ch) || ch == '_' || ch == '-' || ch >= 128)) return false;
    }
    return true;
}

inline std::filesystem::path prepare_output(const std::string& directory,
                                            const std::string& name,
                                            const std::string& explicit_path = "") {
    namespace fs = std::filesystem;
    fs::path output;
    if (explicit_path.empty()) {
        if (!valid_name(name))
            throw std::runtime_error("map_name must contain only letters, digits, '_' or '-' (no extension or path separators).");
        const auto now = std::chrono::system_clock::now();
        const auto seconds = std::chrono::system_clock::to_time_t(now);
        std::tm local{};
        localtime_r(&seconds, &local);
        const auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count() % 1000000;
        std::ostringstream filename;
        filename << std::put_time(&local, "%Y%m%d_%H%M%S_")
                 << std::setw(6) << std::setfill('0') << microseconds << '_' << name << ".pcd";
        output = fs::path(directory) / filename.str();
    } else {
        output = explicit_path;
    }
    output = fs::absolute(output).lexically_normal();
    if (output.extension() != ".pcd")
        throw std::runtime_error("Map output must have a .pcd extension.");
    if (fs::exists(output))
        throw std::runtime_error("Map output already exists; refusing to overwrite a previous session: " + output.string());
    fs::create_directories(output.parent_path());
    return output;
}

template<typename PointT>
bool write_atomic(const std::filesystem::path& output,
                  const pcl::PointCloud<PointT>& cloud, std::string& message) {
    if (cloud.empty()) {
        message = "No mapping points available; no PCD written.";
        return false;
    }
    std::filesystem::path temporary;
    try {
        std::filesystem::create_directories(output.parent_path());
        std::string pattern = output.string() + ".tmpXXXXXX";
        std::vector<char> writable(pattern.begin(), pattern.end());
        writable.push_back('\0');
        const int fd = mkstemp(writable.data());
        if (fd < 0) throw std::runtime_error("Cannot create temporary PCD beside " + output.string());
        close(fd);
        temporary = writable.data();
        pcl::PCDWriter writer;
        if (writer.writeBinary(temporary.string(), cloud) < 0)
            throw std::runtime_error("PCD writer failed.");
        // Replace this session's snapshot only after a complete PCD is written.
        std::filesystem::rename(temporary, output);
        message = "Map saved: " + output.string();
        return true;
    } catch (const std::exception& error) {
        if (!temporary.empty()) {
            std::error_code ignored;
            std::filesystem::remove(temporary, ignored);
        }
        message = std::string("Map save failed: ") + error.what();
        return false;
    }
}
} // namespace fastlio_maps
