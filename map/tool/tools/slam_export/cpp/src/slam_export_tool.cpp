#include "slam_export/slam_export_tool.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <minizip/unzip.h>

namespace autodrive::slam_export {
namespace {

double round3(double value) {
    return std::round(value * 1000.0) / 1000.0;
}

std::string format3(double value) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << value;
    return out.str();
}

std::string base_name(const std::string& file_name) {
    const auto index = file_name.find_last_of('.');
    return index == std::string::npos ? file_name : file_name.substr(0, index);
}

std::string replace_extension(const std::string& file_name, const std::string& extension) {
    return base_name(file_name) + extension;
}

std::vector<unsigned char> read_zip_entry(unzFile archive, const char* name) {
    if (unzLocateFile(archive, name, 0) != UNZ_OK) {
        throw std::runtime_error(std::string("missing zip entry: ") + name);
    }
    unz_file_info64 info {};
    if (unzGetCurrentFileInfo64(archive, &info, nullptr, 0, nullptr, 0, nullptr, 0) != UNZ_OK) {
        throw std::runtime_error("failed to stat zip entry");
    }
    if (unzOpenCurrentFile(archive) != UNZ_OK) {
        throw std::runtime_error("failed to open zip entry");
    }
    std::vector<unsigned char> bytes(static_cast<std::size_t>(info.uncompressed_size));
    const auto read_size = unzReadCurrentFile(archive, bytes.data(), static_cast<unsigned int>(bytes.size()));
    unzCloseCurrentFile(archive);
    if (read_size < 0 || static_cast<std::size_t>(read_size) != bytes.size()) {
        throw std::runtime_error("failed to read zip entry");
    }
    return bytes;
}

PgmMetadata build_pgm(const nlohmann::json& manifest, const std::vector<SlamPoint>& points, double resolution, int padding_cells) {
    const auto browser = manifest.contains("browser_occupancy") && manifest["browser_occupancy"].is_object()
        ? manifest["browser_occupancy"]
        : nlohmann::json::object();
    const double occupancy_voxel = std::max(0.02, browser.value("voxel_size", resolution));
    const bool has_occupied_cells = browser.contains("occupied_cells") && browser["occupied_cells"].is_array() && !browser["occupied_cells"].empty();

    long min_cell_x = std::numeric_limits<long>::max();
    long max_cell_x = std::numeric_limits<long>::min();
    long min_cell_y = std::numeric_limits<long>::max();
    long max_cell_y = std::numeric_limits<long>::min();
    std::set<std::pair<long, long>> occupied;

    if (has_occupied_cells) {
        for (const auto& cell : browser["occupied_cells"]) {
            const long ix = std::lround(cell.value("ix", 0.0));
            const long iy = std::lround(cell.value("iy", 0.0));
            min_cell_x = std::min(min_cell_x, ix);
            max_cell_x = std::max(max_cell_x, ix);
            min_cell_y = std::min(min_cell_y, iy);
            max_cell_y = std::max(max_cell_y, iy);
            occupied.insert({ix, iy});
        }
    } else {
        if (points.empty()) {
            throw std::runtime_error("No radar points in SLAM");
        }
        for (const auto& point : points) {
            const long ix = std::lround(static_cast<double>(point.x) / resolution);
            const long iy = std::lround(static_cast<double>(point.y) / resolution);
            min_cell_x = std::min(min_cell_x, ix);
            max_cell_x = std::max(max_cell_x, ix);
            min_cell_y = std::min(min_cell_y, iy);
            max_cell_y = std::max(max_cell_y, iy);
            occupied.insert({ix, iy});
        }
    }

    const long padded_min_x = min_cell_x - padding_cells;
    const long padded_min_y = min_cell_y - padding_cells;
    const long padded_max_x = max_cell_x + padding_cells;
    const long padded_max_y = max_cell_y + padding_cells;
    const int width = std::max(1L, padded_max_x - padded_min_x + 1);
    const int height = std::max(1L, padded_max_y - padded_min_y + 1);
    std::vector<int> grid(static_cast<std::size_t>(width * height), 205);

    for (const auto& [ix_raw, iy_raw] : occupied) {
        const int ix = static_cast<int>(ix_raw - padded_min_x);
        const int iy = static_cast<int>(iy_raw - padded_min_y);
        const int flipped_y = height - 1 - iy;
        grid[static_cast<std::size_t>(flipped_y * width + ix)] = 0;
    }

    std::ostringstream pgm;
    pgm << "P2\n# Generated from SLAM occupancy\n" << width << " " << height << "\n255\n";
    for (int row = 0; row < height; ++row) {
        if (row > 0) {
            pgm << "\n";
        }
        const int start = row * width;
        for (int col = 0; col < width; ++col) {
            if (col > 0) {
                pgm << " ";
            }
            pgm << grid[static_cast<std::size_t>(start + col)];
        }
    }
    pgm << "\n";

    return PgmMetadata{
        pgm.str(),
        width,
        height,
        {round3(padded_min_x * occupancy_voxel), round3(padded_min_y * occupancy_voxel), 0.0},
        static_cast<int>(occupied.size()),
        {
            {"minX", round3(min_cell_x * occupancy_voxel)},
            {"maxX", round3(max_cell_x * occupancy_voxel)},
            {"minY", round3(min_cell_y * occupancy_voxel)},
            {"maxY", round3(max_cell_y * occupancy_voxel)},
        }
    };
}

std::string build_yaml(const std::string& file_name, double resolution, const std::array<double, 3>& origin) {
    std::ostringstream out;
    out << "image: " << replace_extension(file_name, ".pgm") << "\n";
    out << "mode: trinary\n";
    out << "resolution: " << format3(resolution) << "\n";
    out << "origin: [" << format3(origin[0]) << ", " << format3(origin[1]) << ", " << static_cast<int>(std::llround(origin[2])) << "]\n";
    out << "negate: 0\n";
    out << "occupied_thresh: 0.65\n";
    out << "free_thresh: 0.196";
    return out.str();
}

}  // namespace

LoadedSlam SlamExportTool::load(const std::string& path) {
    unzFile archive = unzOpen64(path.c_str());
    if (archive == nullptr) {
        throw std::runtime_error("failed to open .slam archive");
    }
    try {
        const auto manifest_bytes = read_zip_entry(archive, "manifest.json");
        const auto radar_bytes = read_zip_entry(archive, "radar_points.bin");
        unzClose(archive);

        const auto manifest_text = std::string(manifest_bytes.begin(), manifest_bytes.end());
        return LoadedSlam{nlohmann::json::parse(manifest_text), parse_radar_points(radar_bytes)};
    } catch (...) {
        unzClose(archive);
        throw;
    }
}

std::vector<SlamPoint> SlamExportTool::parse_radar_points(const std::vector<unsigned char>& bytes) {
    std::vector<SlamPoint> points;
    for (std::size_t offset = 0; offset + 12 <= bytes.size(); offset += 12) {
        union {
            std::uint32_t raw;
            float value;
        } x {}, y {}, intensity {};
        x.raw = static_cast<std::uint32_t>(bytes[offset])
            | (static_cast<std::uint32_t>(bytes[offset + 1]) << 8)
            | (static_cast<std::uint32_t>(bytes[offset + 2]) << 16)
            | (static_cast<std::uint32_t>(bytes[offset + 3]) << 24);
        y.raw = static_cast<std::uint32_t>(bytes[offset + 4])
            | (static_cast<std::uint32_t>(bytes[offset + 5]) << 8)
            | (static_cast<std::uint32_t>(bytes[offset + 6]) << 16)
            | (static_cast<std::uint32_t>(bytes[offset + 7]) << 24);
        intensity.raw = static_cast<std::uint32_t>(bytes[offset + 8])
            | (static_cast<std::uint32_t>(bytes[offset + 9]) << 8)
            | (static_cast<std::uint32_t>(bytes[offset + 10]) << 16)
            | (static_cast<std::uint32_t>(bytes[offset + 11]) << 24);
        points.push_back({x.value, y.value, intensity.value});
    }
    return points;
}

ExportArtifacts SlamExportTool::build_exports(
    const std::string& source_file,
    const nlohmann::json& manifest,
    const std::vector<SlamPoint>& points,
    double resolution,
    int padding_cells
) {
    const auto pgm = build_pgm(manifest, points, resolution, padding_cells);
    const auto yaml = build_yaml(source_file, resolution, pgm.origin);

    auto export_manifest = manifest;
    export_manifest.erase("browser_occupancy");
    export_manifest.erase("trajectory");

    nlohmann::json export_json = {
        {"source_file", source_file},
        {"map_yaml", {
            {"image", replace_extension(source_file, ".pgm")},
            {"mode", "trinary"},
            {"resolution", resolution},
            {"origin", {pgm.origin[0], pgm.origin[1], pgm.origin[2]}},
            {"negate", 0},
            {"occupied_thresh", 0.65},
            {"free_thresh", 0.196},
        }},
        {"pgm_meta", {
            {"width", pgm.width},
            {"height", pgm.height},
            {"occupied_cells", pgm.occupied_cells},
            {"bounds", pgm.bounds},
        }},
        {"manifest", export_manifest},
    };

    return ExportArtifacts{pgm, yaml, export_json.dump(2)};
}

ExportArtifacts SlamExportTool::export_to_dir(
    const std::string& slam_path,
    const std::string& output_dir,
    double resolution,
    int padding_cells
) {
    const auto loaded = load(slam_path);
    const auto source_file = std::filesystem::path(slam_path).filename().string();
    const auto artifacts = build_exports(source_file, loaded.manifest, loaded.radar_points, resolution, padding_cells);
    std::filesystem::create_directories(output_dir);
    const auto stem = base_name(source_file);
    std::ofstream(output_dir + "/" + stem + ".pgm") << artifacts.pgm.pgm_text;
    std::ofstream(output_dir + "/" + stem + ".yaml") << artifacts.yaml_text;
    std::ofstream(output_dir + "/" + stem + ".json") << artifacts.json_text;
    return artifacts;
}

}  // namespace autodrive::slam_export
