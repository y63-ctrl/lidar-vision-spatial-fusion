#include "pipeline/Pipeline.h"

#include "degradation/DegradationEvaluator.h"
#include "fusion/FusionStrategyFactory.h"
#include "io/DegradationJsonWriter.h"
#include "io/ImageWriter.h"
#include "io/OccupancyMapWriter.h"
#include "io/PcdWriter.h"
#include "io/TimingJsonWriter.h"
#include "mapping/BevOccupancyGrid.h"
#include "mapping/PointCloudPreprocessor.h"
#include "viz/OccupancyMapRenderer.h"
#include "viz/OverlayRenderer.h"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <utility>

namespace lvf {
namespace {

double elapsedMs(std::chrono::steady_clock::time_point start,
                 std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

}  // namespace

Pipeline::Pipeline(PipelineConfig config) : config_(std::move(config)) {}

FrameRunResult Pipeline::run(const Frame& frame, double readMs) const {
    FrameRunResult result;
    result.frame_id = frame.id;
    result.timing.read_ms = readMs;

    const auto runStart = std::chrono::steady_clock::now();

    const auto degradationStart = std::chrono::steady_clock::now();
    DegradationEvaluator evaluator;
    const DegradationReport degradation = evaluator.evaluate(frame);
    const auto degradationEnd = std::chrono::steady_clock::now();
    result.degradation = degradation;
    result.timing.degradation_ms = elapsedMs(degradationStart, degradationEnd);

    const auto fusionStart = std::chrono::steady_clock::now();
    FusionOptions fusionOptions;
    fusionOptions.min_depth = config_.min_depth;
    fusionOptions.max_depth = config_.max_depth;
    fusionOptions.threads = config_.threads;

    std::unique_ptr<IFusionStrategy> strategy;
    if (config_.fusion_mode == "auto") {
        strategy = FusionStrategyFactory::createForAction(degradation.action, fusionOptions);
    } else {
        strategy = FusionStrategyFactory::create(config_.fusion_mode, fusionOptions);
    }
    const FusionResult fusion = strategy->fuse(frame);
    const auto fusionEnd = std::chrono::steady_clock::now();
    result.timing.fusion_ms = elapsedMs(fusionStart, fusionEnd);
    result.input_points = fusion.input_points;
    result.projected_points = fusion.projected_points;
    result.colored_points = fusion.colored_points;

    const auto mappingStart = std::chrono::steady_clock::now();
    MappingOptions mappingOptions;
    mappingOptions.x_min = config_.x_min;
    mappingOptions.x_max = config_.x_max;
    mappingOptions.y_min = config_.y_min;
    mappingOptions.y_max = config_.y_max;
    mappingOptions.z_min = config_.z_min;
    mappingOptions.z_max = config_.z_max;
    mappingOptions.voxel_size = config_.voxel_size;

    PointCloudPreprocessor preprocessor(mappingOptions);
    const PreprocessResult preprocessed = preprocessor.process(frame.cloud);

    BevOptions bevOptions;
    bevOptions.x_min = config_.x_min;
    bevOptions.x_max = config_.x_max;
    bevOptions.y_min = config_.y_min;
    bevOptions.y_max = config_.y_max;
    bevOptions.resolution = config_.resolution;
    bevOptions.raycast = config_.raycast;
    bevOptions.inflation_cells = config_.inflation_cells;

    BevOccupancyGrid mapper(bevOptions);
    const BevResult bev = mapper.build(*preprocessed.cloud);
    const auto mappingEnd = std::chrono::steady_clock::now();
    result.timing.mapping_ms = elapsedMs(mappingStart, mappingEnd);
    result.occupied_cells = bev.stats.occupied;
    result.free_cells = bev.stats.free;
    result.unknown_cells = bev.stats.unknown;

    const auto writeStart = std::chrono::steady_clock::now();
    std::filesystem::create_directories(config_.output_dir);
    const std::string& stem = frame.id;

    const auto degradationPath = config_.output_dir / (stem + "_degradation.json");
    writeDegradationJson(degradationPath, degradation, frame.id);
    result.outputs.push_back(degradationPath);

    if (config_.write_pcd && fusion.cloud != nullptr && !fusion.cloud->empty()) {
        const auto pcdPath = config_.output_dir / (stem + "_fused.pcd");
        writeColoredCloud(pcdPath, *fusion.cloud);
        result.outputs.push_back(pcdPath);
    }

    if (config_.make_visualization) {
        OverlayOptions overlayOptions;
        overlayOptions.min_depth = config_.min_depth;
        overlayOptions.max_depth = config_.max_depth;
        overlayOptions.point_size = 2;
        overlayOptions.draw_text = true;

        const cv::Mat overlay = OverlayRenderer::renderOverlay(frame, fusion, overlayOptions);
        const auto overlayPath = config_.output_dir / (stem + "_overlay.png");
        writePng(overlayPath, overlay);
        result.outputs.push_back(overlayPath);

        const cv::Mat fusedView =
            OverlayRenderer::renderFusedView(frame, fusion, overlayOptions);
        const auto fusedViewPath = config_.output_dir / (stem + "_fused_view.png");
        writePng(fusedViewPath, fusedView);
        result.outputs.push_back(fusedViewPath);
    }

    OccupancyMapMetadata metadata;
    metadata.frameId = frame.id;
    metadata.heightMin = config_.z_min;
    metadata.heightMax = config_.z_max;
    metadata.raycast = config_.raycast;
    metadata.inflationCells = config_.inflation_cells;
    writeOccupancyMap(config_.output_dir, stem, bev.grid, metadata);
    result.outputs.push_back(config_.output_dir / (stem + "_occupancy.pgm"));
    result.outputs.push_back(config_.output_dir / (stem + "_occupancy.yaml"));

    if (config_.make_visualization) {
        OccupancyRenderOptions renderOptions;
        renderOptions.scale = config_.preview_scale;
        renderOptions.draw_grid = config_.draw_grid;
        const cv::Mat preview = renderOccupancyMap(bev.grid, renderOptions);
        const auto previewPath = config_.output_dir / (stem + "_occupancy.png");
        writePng(previewPath, preview);
        result.outputs.push_back(previewPath);
    }

    const auto writeEnd = std::chrono::steady_clock::now();
    result.timing.write_ms = elapsedMs(writeStart, writeEnd);
    result.timing.compute_ms = result.timing.degradation_ms + result.timing.fusion_ms +
                               result.timing.mapping_ms;
    result.timing.total_ms = readMs + elapsedMs(runStart, writeEnd);

    const auto timingPath = config_.output_dir / (stem + "_timing.json");
    writeTimingJson(timingPath, result.timing, frame.id);
    result.outputs.push_back(timingPath);

    return result;
}

}  // namespace lvf
