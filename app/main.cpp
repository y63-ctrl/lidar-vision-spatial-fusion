#include "core/Version.h"
#include "data/KittiDataSource.h"
#include "pipeline/Pipeline.h"
#include "utils/Log.h"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
    lvf::PipelineConfig config;
    bool all = false;
};

void printUsage() {
    std::cout
        << "Usage: lidar_fusion [options]\n"
        << "  --data-root <dir>    KITTI-style data root (default: data/sample)\n"
        << "  --frame <id>         single frame id (default: 000000)\n"
        << "  --all                process all frames discovered in image_2/\n"
        << "  --out <dir>          output directory (default: results)\n"
        << "  --calib <file>       optional calibration override\n"
        << "  --mode <name>        auto | pinhole | depth (default: auto)\n"
        << "  --threads <n>        OpenMP threads, 0 = default\n"
        << "  --resolution <m>     BEV resolution (default: 0.2)\n"
        << "  --voxel <m>          voxel leaf size (default: 0.2)\n"
        << "  --x-min/--x-max/--y-min/--y-max/--z-min/--z-max\n"
        << "  --min-depth <m>      projection min depth (default: 0.5)\n"
        << "  --max-depth <m>      projection max depth (default: 80)\n"
        << "  --inflation <n>      BEV obstacle inflation cells (default: 0)\n"
        << "  --no-raycast         disable free-space ray marking\n"
        << "  --no-viz             skip overlay/fused view/occupancy preview\n"
        << "  --no-pcd             skip fused PCD output\n"
        << "  --no-grid            do not draw 10m grid lines in preview\n"
        << "  --preview-scale <n>  occupancy preview scale (default: 2)\n"
        << "  --fail-on-degraded   return exit code 3 when a modality is degraded\n"
        << "  --version            print version and exit\n"
        << "  --help               show this message\n";
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
            options.config.data_root = next("--data-root");
        } else if (arg == "--frame") {
            options.config.frame_id = next("--frame");
        } else if (arg == "--all") {
            options.all = true;
        } else if (arg == "--out") {
            options.config.output_dir = next("--out");
        } else if (arg == "--calib") {
            options.config.calib_override = next("--calib");
        } else if (arg == "--mode") {
            options.config.fusion_mode = next("--mode");
            if (options.config.fusion_mode != "auto" &&
                options.config.fusion_mode != "pinhole" &&
                options.config.fusion_mode != "depth") {
                throw std::invalid_argument("--mode must be auto, pinhole or depth");
            }
        } else if (arg == "--threads") {
            options.config.threads = std::stoi(next("--threads"));
        } else if (arg == "--resolution") {
            options.config.resolution = std::stof(next("--resolution"));
        } else if (arg == "--voxel") {
            options.config.voxel_size = std::stof(next("--voxel"));
        } else if (arg == "--x-min") {
            options.config.x_min = std::stof(next("--x-min"));
        } else if (arg == "--x-max") {
            options.config.x_max = std::stof(next("--x-max"));
        } else if (arg == "--y-min") {
            options.config.y_min = std::stof(next("--y-min"));
        } else if (arg == "--y-max") {
            options.config.y_max = std::stof(next("--y-max"));
        } else if (arg == "--z-min") {
            options.config.z_min = std::stof(next("--z-min"));
        } else if (arg == "--z-max") {
            options.config.z_max = std::stof(next("--z-max"));
        } else if (arg == "--min-depth") {
            options.config.min_depth = std::stof(next("--min-depth"));
        } else if (arg == "--max-depth") {
            options.config.max_depth = std::stof(next("--max-depth"));
        } else if (arg == "--inflation") {
            options.config.inflation_cells = std::stoi(next("--inflation"));
        } else if (arg == "--no-raycast") {
            options.config.raycast = false;
        } else if (arg == "--no-viz") {
            options.config.make_visualization = false;
        } else if (arg == "--no-pcd") {
            options.config.write_pcd = false;
        } else if (arg == "--no-grid") {
            options.config.draw_grid = false;
        } else if (arg == "--preview-scale") {
            options.config.preview_scale = std::stoi(next("--preview-scale"));
        } else if (arg == "--fail-on-degraded") {
            options.config.fail_on_degraded = true;
        } else if (arg == "--version") {
            std::cout << lvf::version() << '\n';
            std::exit(0);
        } else if (arg == "--help" || arg == "-h") {
            printUsage();
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown argument: " + arg);
        }
    }

    if (options.config.threads < 0) {
        throw std::invalid_argument("--threads must be non-negative");
    }
    if (options.config.preview_scale <= 0) {
        throw std::invalid_argument("--preview-scale must be positive");
    }
    return options;
}

void writeSummaryHeader(std::ostream& out) {
    out << "frame,input_points,projected_points,colored_points,action,overall,"
           "image_confidence,lidar_confidence,map_trusted,occupied,free,unknown,"
           "read_ms,degradation_ms,fusion_ms,mapping_ms,write_ms,compute_ms,total_ms\n";
}

void writeSummaryRow(std::ostream& out, const lvf::FrameRunResult& result) {
    out << std::fixed << std::setprecision(4)
        << result.frame_id << ','
        << result.input_points << ','
        << result.projected_points << ','
        << result.colored_points << ','
        << result.degradation.action << ','
        << result.degradation.overall << ','
        << result.degradation.image.confidence << ','
        << result.degradation.lidar.confidence << ','
        << (result.degradation.map_trusted ? 1 : 0) << ','
        << result.occupied_cells << ','
        << result.free_cells << ','
        << result.unknown_cells << ','
        << result.timing.read_ms << ','
        << result.timing.degradation_ms << ','
        << result.timing.fusion_ms << ','
        << result.timing.mapping_ms << ','
        << result.timing.write_ms << ','
        << result.timing.compute_ms << ','
        << result.timing.total_ms << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    try {
        options = parseOptions(argc, argv);
    } catch (const std::exception& error) {
        lvf::logError(error.what());
        printUsage();
        return 1;
    }

    try {
        std::filesystem::create_directories(options.config.output_dir);
        const auto summaryPath = options.config.output_dir / "summary.csv";
        std::ofstream summary(summaryPath, std::ios::trunc);
        if (!summary) {
            throw std::runtime_error("cannot open " + summaryPath.string());
        }
        writeSummaryHeader(summary);

        const std::vector<std::string> frameIds =
            options.all ? std::vector<std::string>{}
                        : std::vector<std::string>{options.config.frame_id};
        lvf::KittiDataSource source(options.config.data_root, frameIds,
                                    options.config.calib_override);
        lvf::Pipeline pipeline(options.config);

        lvf::Frame frame;
        std::size_t frameCount = 0;
        bool blocked = false;

        while (true) {
            const auto readStart = std::chrono::steady_clock::now();
            if (!source.next(frame)) {
                break;
            }
            const auto readEnd = std::chrono::steady_clock::now();
            const double readMs =
                std::chrono::duration<double, std::milli>(readEnd - readStart).count();

            const auto result = pipeline.run(frame, readMs);
            writeSummaryRow(summary, result);

            lvf::logInfo("frame " + result.frame_id + " | " + result.degradation.action +
                         " | points " + std::to_string(result.input_points) + "/" +
                         std::to_string(result.colored_points) + " | grid " +
                         std::to_string(result.occupied_cells) + "/" +
                         std::to_string(result.free_cells) + "/" +
                         std::to_string(result.unknown_cells) + " | " +
                         std::to_string(static_cast<int>(result.timing.total_ms + 0.5)) + " ms");

            ++frameCount;
            if (options.config.fail_on_degraded &&
                (result.degradation.image.degraded || result.degradation.lidar.degraded)) {
                blocked = true;
                break;
            }
        }

        if (frameCount == 0) {
            throw std::runtime_error("no frames were processed");
        }

        summary.flush();
        if (!summary) {
            throw std::runtime_error("failed to write " + summaryPath.string());
        }

        lvf::logInfo("processed " + std::to_string(frameCount) + " frame(s); outputs in " +
                     options.config.output_dir.filename().string());
        if (blocked) {
            lvf::logWarn("degradation blocked by --fail-on-degraded");
            return 3;
        }
        return 0;
    } catch (const std::exception& error) {
        lvf::logError(error.what());
        return 2;
    }
}
