#include "map_storage.hpp"
#include <pcl/point_types.h>
#include <iostream>
#include <regex>

static void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

int main() {
    namespace fs = std::filesystem;
    char pattern[] = "/tmp/fastlio-map-tests-XXXXXX";
    char* created = mkdtemp(pattern);
    if (!created) return 1;
    const fs::path root(created);
    try {
        const auto output = fastlio_maps::prepare_output((root / "pcd_map").string(), "lab_a");
        require(output.parent_path() == root / "pcd_map", "Output is not in map directory");
        require(std::regex_match(output.filename().string(),
                std::regex("[0-9]{8}_[0-9]{6}_[0-9]{6}_lab_a\\.pcd")), "Unexpected map filename");
        require(!fs::exists(output), "Prepare wrote an empty map");
        require(fastlio_maps::valid_name("场景A"), "Unicode scene names rejected");
        require(!fastlio_maps::valid_name("../lab"), "Path traversal scene name accepted");
        pcl::PointCloud<pcl::PointXYZI> cloud;
        std::string message;
        require(!fastlio_maps::write_atomic(output, cloud, message), "Empty map reported success");
        require(!fs::exists(output), "Empty map created a file");
        pcl::PointXYZI point{};
        point.x = 1; point.y = 2; point.z = 3; point.intensity = 4;
        cloud.push_back(point);
        require(fastlio_maps::write_atomic(output, cloud, message), "Atomic PCD save failed");
        pcl::PointCloud<pcl::PointXYZI> loaded;
        require(pcl::io::loadPCDFile(output.string(), loaded) == 0 && loaded.size() == 1, "PCD roundtrip failed");
        cloud.push_back(point);
        require(fastlio_maps::write_atomic(output, cloud, message), "Same-session snapshot update failed");
        require(pcl::io::loadPCDFile(output.string(), loaded) == 0 && loaded.size() == 2, "Snapshot did not update");
        bool rejected = false;
        try { fastlio_maps::prepare_output(root.string(), "map", output.string()); }
        catch (const std::exception&) { rejected = true; }
        require(rejected, "Previous session output was allowed to overwrite");
        const auto blocked = root / "blocked.pcd";
        fs::create_directory(blocked);
        require(!fastlio_maps::write_atomic(blocked, cloud, message), "Failed rename reported success");
        require(fs::is_directory(blocked), "Failed save destroyed existing target");
        for (const auto& entry : fs::directory_iterator(root))
            require(entry.path().filename().string().find(".tmp") == std::string::npos, "Temporary PCD leaked");
        fs::remove_all(root);
        std::cout << "PASS: scene naming, empty/save errors, PCD roundtrip, snapshot update and overwrite protection\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        fs::remove_all(root);
        return 1;
    }
}
