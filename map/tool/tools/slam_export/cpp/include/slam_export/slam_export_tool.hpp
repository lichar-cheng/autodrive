#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace autodrive::slam_export {

struct SlamPoint {
    float x {};
    float y {};
    float intensity {};
};

struct LoadedSlam {
    nlohmann::json manifest;
    std::vector<SlamPoint> radar_points;
};

struct PgmMetadata {
    std::string pgm_text;
    int width {};
    int height {};
    std::array<double, 3> origin {};
    int occupied_cells {};
    std::map<std::string, double> bounds;
};

struct ExportArtifacts {
    PgmMetadata pgm;
    std::string yaml_text;
    std::string json_text;
};

class SlamExportTool {
public:
    static LoadedSlam load(const std::string& path);
    static std::vector<SlamPoint> parse_radar_points(const std::vector<unsigned char>& bytes);
    static ExportArtifacts build_exports(
        const std::string& source_file,
        const nlohmann::json& manifest,
        const std::vector<SlamPoint>& points,
        double resolution,
        int padding_cells
    );
    static ExportArtifacts export_to_dir(
        const std::string& slam_path,
        const std::string& output_dir,
        double resolution,
        int padding_cells
    );
};

}  // namespace autodrive::slam_export
