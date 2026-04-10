#include "slam_export/slam_export_tool.hpp"

#include <cassert>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <minizip/zip.h>

using autodrive::slam_export::SlamExportTool;
using autodrive::slam_export::SlamPoint;

namespace {

std::vector<unsigned char> encode_points(const std::vector<SlamPoint>& points) {
    std::vector<unsigned char> bytes;
    bytes.reserve(points.size() * 12);
    for (const auto& point : points) {
        const float values[3] = {point.x, point.y, point.intensity};
        for (float value : values) {
            union {
                float value;
                std::uint32_t raw;
            } bits {};
            bits.value = value;
            bytes.push_back(static_cast<unsigned char>(bits.raw & 0xff));
            bytes.push_back(static_cast<unsigned char>((bits.raw >> 8) & 0xff));
            bytes.push_back(static_cast<unsigned char>((bits.raw >> 16) & 0xff));
            bytes.push_back(static_cast<unsigned char>((bits.raw >> 24) & 0xff));
        }
    }
    return bytes;
}

void add_entry(zipFile archive, const char* name, const std::string& text) {
    zip_fileinfo info {};
    assert(zipOpenNewFileInZip(archive, name, &info, nullptr, 0, nullptr, 0, nullptr, Z_DEFLATED, Z_DEFAULT_COMPRESSION) == ZIP_OK);
    assert(zipWriteInFileInZip(archive, text.data(), static_cast<unsigned int>(text.size())) == ZIP_OK);
    assert(zipCloseFileInZip(archive) == ZIP_OK);
}

void add_entry(zipFile archive, const char* name, const std::vector<unsigned char>& bytes) {
    zip_fileinfo info {};
    assert(zipOpenNewFileInZip(archive, name, &info, nullptr, 0, nullptr, 0, nullptr, Z_DEFLATED, Z_DEFAULT_COMPRESSION) == ZIP_OK);
    assert(zipWriteInFileInZip(archive, bytes.data(), static_cast<unsigned int>(bytes.size())) == ZIP_OK);
    assert(zipCloseFileInZip(archive) == ZIP_OK);
}

void create_slam(const std::filesystem::path& path, const std::string& manifest_json, const std::vector<SlamPoint>& points) {
    zipFile archive = zipOpen64(path.string().c_str(), APPEND_STATUS_CREATE);
    assert(archive != nullptr);
    add_entry(archive, "manifest.json", manifest_json);
    add_entry(archive, "radar_points.bin", encode_points(points));
    assert(zipClose(archive, nullptr) == ZIP_OK);
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream in(path);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

void test_load_and_build_from_browser_occupancy() {
    const auto dir = std::filesystem::temp_directory_path() / "slam_cpp_test_a";
    std::filesystem::create_directories(dir);
    const auto slam_path = dir / "demo.slam";
    create_slam(
        slam_path,
        R"({"version":"stcm.v2","trajectory":[{"x":1}],"browser_occupancy":{"voxel_size":0.2,"occupied_cells":[{"ix":0,"iy":0},{"ix":1,"iy":0}]},"poi":[{"name":"A","x":1.0,"y":2.0}]})",
        {{0.0f, 0.0f, 1.0f}}
    );

    const auto loaded = SlamExportTool::load(slam_path.string());
    const auto artifacts = SlamExportTool::build_exports("demo.slam", loaded.manifest, loaded.radar_points, 0.1, 1);

    assert(artifacts.pgm.pgm_text.rfind("P2\n# Generated from SLAM occupancy\n4 3\n255\n", 0) == 0);
    assert(artifacts.yaml_text.find("image: demo.pgm") != std::string::npos);
    assert(artifacts.yaml_text.find("origin: [-0.200, -0.200, 0]") != std::string::npos);
    assert(artifacts.json_text.find("\"source_file\": \"demo.slam\"") != std::string::npos);
    assert(artifacts.json_text.find("\"occupied_cells\": 2") != std::string::npos);
    assert(artifacts.json_text.find("browser_occupancy") == std::string::npos);
    assert(artifacts.json_text.find("\"trajectory\"") == std::string::npos);
}

void test_fallback_to_points_and_write_files() {
    const auto dir = std::filesystem::temp_directory_path() / "slam_cpp_test_b";
    std::filesystem::create_directories(dir);
    const auto slam_path = dir / "points_only.slam";
    create_slam(
        slam_path,
        R"({"version":"stcm.v2","poi":[{"name":"B","x":0.0,"y":0.0}]})",
        {{0.0f, 0.0f, 1.0f}, {0.1f, 0.0f, 1.0f}}
    );

    const auto out_dir = dir / "out";
    const auto artifacts = SlamExportTool::export_to_dir(slam_path.string(), out_dir.string(), 0.1, 1);

    assert(artifacts.pgm.occupied_cells == 2);
    assert(std::filesystem::exists(out_dir / "points_only.pgm"));
    assert(std::filesystem::exists(out_dir / "points_only.yaml"));
    assert(std::filesystem::exists(out_dir / "points_only.json"));
    assert(read_file(out_dir / "points_only.yaml").find("resolution: 0.100") != std::string::npos);
}

}  // namespace

int main() {
    test_load_and_build_from_browser_occupancy();
    test_fallback_to_points_and_write_files();
    return 0;
}
