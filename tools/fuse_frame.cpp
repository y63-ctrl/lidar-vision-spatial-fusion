#include "data/KittiDataSource.h"
#include "degradation/DegradationEvaluator.h"
#include "fusion/FusionOptions.h"
#include "fusion/FusionStrategyFactory.h"
#include "io/DegradationJsonWriter.h"
#include "io/ImageWriter.h"
#include "io/PcdWriter.h"
#include "viz/OverlayRenderer.h"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

struct Options {
    std::filesystem::path dataRoot{"data/sample"};
    std::filesystem::path outputDir{"results"};
    std::filesystem::path calibOverride;
    std::string frameId{"000000"};
    std::string mode{"auto"};
    float minDepth = 0.5f;
    float maxDepth = 80.0f;
    int threads = 0;
    int pointSize = 2;
    bool drawText = true;
};

void printUsage() {
    std::cout
        << "Usage: lvf_fuse_frame [options]\n"
        << "  --data-root <dir>   KITTI-style data root (default: data/sample)\n"
        << "  --frame <id>        frame id (default: 000000)\n"
        << "  --calib <file>      optional calibration override\n"
        << "  --out <dir>         output directory (default: results)\n"
        << "  --mode <name>       auto | pinhole | depth (default: auto)\n"
        << "  --threads <n>       OpenMP threads, 0 = default\n"
        << "  --min-depth <m>     minimum projection depth (default: 0.5)\n"
        << "  --max-depth <m>     maximum projection depth (default: 80)\n"
        << "  --point-size <px>   overlay point size (default: 2)\n"
        << "  --no-text           do not draw the frame label\n"
        << "  --help\n";
}

Options parseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                throw std::invalid_argument(std::string("missing value for ") + name);
            }
            return argv[++i];
        };

        if (arg == "--data-root") {
            options.dataRoot = next("--data-root");
        } else if (arg == "--frame") {
            options.frameId = next("--frame");
        } else if (arg == "--calib") {
            options.calibOverride = next("--calib");
        } else if (arg == "--out") {
            options.outputDir = next("--out");
        } else if (arg == "--mode") {
            options.mode = next("--mode");
        } else if (arg == "--threads") {
            options.threads = std::stoi(next("--threads"));
        } else if (arg == "--min-depth") {
            options.minDepth = std::stof(next("--min-depth"));
        } else if (arg == "--max-depth") {
            options.maxDepth = std::stof(next("--max-depth"));
        } else if (arg == "--point-size") {
            options.pointSize = std::stoi(next("--point-size"));
        } else if (arg == "--no-text") {
            options.drawText = false;
        } else if (arg == "--help" || arg == "-h") {
            printUsage();
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown argument: " + arg);
        }
    }
    return options;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseOptions(argc, argv);

        lvf::KittiDataSource source(options.dataRoot, {options.frameId}, options.calibOverride);
        lvf::Frame frame;
        if (!source.next(frame)) {
            throw std::runtime_error("no frame returned by KittiDataSource");
        }

        const auto start = std::chrono::steady_clock::now();

        lvf::DegradationEvaluator evaluator;
        const auto degradationStart = std::chrono::steady_clock::now();
        const lvf::DegradationReport degradation = evaluator.evaluate(frame);
        const auto degradationEnd = std::chrono::steady_clock::now();
        const double degradationMs =
            std::chrono::duration<double, std::milli>(degradationEnd - degradationStart).count();

        lvf::FusionOptions fusionOptions;
        fusionOptions.min_depth = options.minDepth;
        fusionOptions.max_depth = options.maxDepth;
        fusionOptions.threads = options.threads;

        std::string strategyLabel = options.mode;
        std::unique_ptr<lvf::IFusionStrategy> strategy;
        if (options.mode == "auto") {
            strategy = lvf::FusionStrategyFactory::createForAction(degradation.action, fusionOptions);
            strategyLabel = "auto(" + degradation.action + ")";
        } else {
            strategy = lvf::FusionStrategyFactory::create(options.mode, fusionOptions);
        }

        const lvf::FusionResult result = strategy->fuse(frame);

        lvf::OverlayOptions overlayOptions;
        overlayOptions.min_depth = options.minDepth;
        overlayOptions.max_depth = options.maxDepth;
        overlayOptions.point_size = options.pointSize;
        overlayOptions.draw_text = options.drawText;

        const cv::Mat overlay = lvf::OverlayRenderer::renderOverlay(frame, result, overlayOptions);
        const cv::Mat fusedView =
            lvf::OverlayRenderer::renderFusedView(frame, result, overlayOptions);

        std::filesystem::create_directories(options.outputDir);
        const std::string stem = frame.id;
        lvf::writeDegradationJson(options.outputDir / (stem + "_degradation.json"), degradation,
                                  frame.id);
        lvf::writePng(options.outputDir / (stem + "_overlay.png"), overlay);
        lvf::writePng(options.outputDir / (stem + "_fused_view.png"), fusedView);
        lvf::writeColoredCloud(options.outputDir / (stem + "_fused.pcd"), *result.cloud);

        const auto end = std::chrono::steady_clock::now();
        const double elapsedMs =
            std::chrono::duration<double, std::milli>(end - start).count();

        std::cout << "source: " << source.name() << '\n'
                  << "frame: " << frame.id << '\n'
                  << "degradation: " << degradation.action
                  << " (overall=" << degradation.overall << ", " << std::fixed
                  << std::setprecision(2) << degradationMs << " ms)\n"
                  << "strategy: " << strategyLabel << '\n'
                  << "input points: " << result.input_points << '\n'
                  << "projected points: " << result.projected_points << '\n'
                  << "colored points: " << result.colored_points << '\n'
                  << "elapsed: " << std::fixed << std::setprecision(2) << elapsedMs << " ms\n"
                  << "output dir: " << options.outputDir.filename().string() << '\n'
                  << "degradation json: " << stem << "_degradation.json\n"
                  << "overlay: " << stem << "_overlay.png\n"
                  << "fused view: " << stem << "_fused_view.png\n"
                  << "pcd: " << stem << "_fused.pcd\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[ERROR] " << error.what() << '\n';
        return 1;
    }
}
