#include "data/KittiDataSource.h"
#include "pipeline/Pipeline.h"

#include "TestHarness.h"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

namespace {

std::string readText(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    return text;
}

void checkOutputs(const std::vector<std::filesystem::path>& outputs) {
    using namespace lvf::test;
    check(outputs.size() >= 7, "pipeline output count");
    for (const auto& path : outputs) {
        check(std::filesystem::exists(path), "pipeline output exists: " + path.filename().string());
        if (std::filesystem::exists(path)) {
            check(std::filesystem::file_size(path) > 0,
                  "pipeline output non-empty: " + path.filename().string());
        }
    }
}

}  // namespace

int main() {
    using namespace lvf::test;

    const std::filesystem::path sampleDir = LVF_SAMPLE_DIR;
    const auto outputDir = std::filesystem::temp_directory_path() / "lvf_pipeline_e2e";
    std::filesystem::remove_all(outputDir);

    lvf::KittiDataSource source(sampleDir, {"000000"});
    lvf::Frame frame;
    if (!source.next(frame)) {
        std::cerr << "[FAIL] pipeline: sample frame did not load\n";
        return 1;
    }

    lvf::PipelineConfig config;
    config.data_root = sampleDir;
    config.output_dir = outputDir;
    config.frame_id = "000000";
    config.make_visualization = true;
    config.write_pcd = true;

    lvf::Pipeline pipeline(config);
    const lvf::FrameRunResult result = pipeline.run(frame, 0.0);

    check(result.frame_id == "000000", "pipeline frame id");
    check(result.degradation.action == "FULL_FUSION", "pipeline degradation action");
    check(result.degradation.map_trusted, "pipeline map trusted");
    check(result.input_points == 20000, "pipeline input points");
    check(result.colored_points > 0, "pipeline colored points");
    check(result.occupied_cells > 0, "pipeline occupied cells");
    check(result.free_cells > 0, "pipeline free cells");
    check(result.unknown_cells > 0, "pipeline unknown cells");
    check(result.timing.total_ms > 0.0, "pipeline total timing");
    check(result.timing.compute_ms > 0.0, "pipeline compute timing");
    checkOutputs(result.outputs);

    check(std::filesystem::exists(outputDir / "000000_fused.pcd"), "pipeline PCD");
    check(std::filesystem::exists(outputDir / "000000_occupancy.pgm"), "pipeline PGM");
    check(std::filesystem::exists(outputDir / "000000_occupancy.yaml"), "pipeline YAML");
    check(std::filesystem::exists(outputDir / "000000_timing.json"), "pipeline timing json");

    const std::string timing = readText(outputDir / "000000_timing.json");
    check(timing.find("\"frame\": \"000000\"") != std::string::npos, "timing JSON frame");
    check(timing.find("\"read_ms\"") != std::string::npos, "timing JSON read");
    check(timing.find("\"compute_ms\"") != std::string::npos, "timing JSON compute");

    lvf::PipelineConfig noVizConfig = config;
    noVizConfig.output_dir = outputDir / "no_viz";
    noVizConfig.make_visualization = false;
    noVizConfig.write_pcd = false;
    lvf::Pipeline noVizPipeline(noVizConfig);
    const auto noViz = noVizPipeline.run(frame, 0.0);
    check(noViz.outputs.size() == 4, "no-viz/no-pcd output count");
    check(!std::filesystem::exists(noVizConfig.output_dir / "000000_fused.pcd"),
          "no-pcd skips PCD");
    check(!std::filesystem::exists(noVizConfig.output_dir / "000000_overlay.png"),
          "no-viz skips overlay");
    check(std::filesystem::exists(noVizConfig.output_dir / "000000_occupancy.pgm"),
          "no-viz still writes PGM");
    check(std::filesystem::exists(noVizConfig.output_dir / "000000_timing.json"),
          "no-viz still writes timing");

    std::filesystem::remove_all(outputDir);
    return finish("pipeline");
}
