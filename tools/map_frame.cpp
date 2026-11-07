#include "data/KittiDataSource.h"
#include "io/ImageWriter.h"
#include "io/OccupancyMapWriter.h"
#include "mapping/BevOccupancyGrid.h"
#include "mapping/PointCloudPreprocessor.h"
#include "viz/OccupancyMapRenderer.h"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

struct Options {
    std::filesystem::path dataRoot{"data/sample"};
    std::filesystem::path outputDir{"results"};
    std::filesystem::path calibOverride;
    std::string frameId{"000000"};
    float resolution = 0.2f;
    float voxelSize = 0.2f;
    float xMin = -10.0f;
    float xMax = 50.0f;
    float yMin = -20.0f;
    float yMax = 20.0f;
    float zMin = -1.5f;
    float zMax = 2.5f;
    bool raycast = true;
    int inflation = 0;
    int scale = 2;
    bool drawGrid = true;
};

void printUsage() {
    std::cout
        << "Usage: lvf_map_frame [options]\n"
        << "  --data-root <dir>   KITTI-style data root (default: data/sample)\n"
        << "  --frame <id>        frame id (default: 000000)\n"
        << "  --calib <file>      optional calibration override\n"
        << "  --out <dir>         output directory (default: results)\n"
        << "  --resolution <m>    BEV resolution (default: 0.2)\n"
        << "  --voxel <m>         voxel leaf size (default: 0.2)\n"
        << "  --x-min/--x-max/--y-min/--y-max/--z-min/--z-max\n"
        << "  --no-raycast        disable free-space ray marking\n"
        << "  --inflation <n>     obstacle inflation radius in cells (default: 0)\n"
        << "  --scale <n>         preview scale (default: 2)\n"
        << "  --no-grid           do not draw 10m grid lines in preview\n"
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
        } else if (arg == "--resolution") {
            options.resolution = std::stof(next("--resolution"));
        } else if (arg == "--voxel") {
            options.voxelSize = std::stof(next("--voxel"));
        } else if (arg == "--x-min") {
            options.xMin = std::stof(next("--x-min"));
        } else if (arg == "--x-max") {
            options.xMax = std::stof(next("--x-max"));
        } else if (arg == "--y-min") {
            options.yMin = std::stof(next("--y-min"));
        } else if (arg == "--y-max") {
            options.yMax = std::stof(next("--y-max"));
        } else if (arg == "--z-min") {
            options.zMin = std::stof(next("--z-min"));
        } else if (arg == "--z-max") {
            options.zMax = std::stof(next("--z-max"));
        } else if (arg == "--no-raycast") {
            options.raycast = false;
        } else if (arg == "--inflation") {
            options.inflation = std::stoi(next("--inflation"));
        } else if (arg == "--scale") {
            options.scale = std::stoi(next("--scale"));
        } else if (arg == "--no-grid") {
            options.drawGrid = false;
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
        if (options.scale <= 0) {
            throw std::invalid_argument("--scale must be positive");
        }

        lvf::KittiDataSource source(options.dataRoot, {options.frameId}, options.calibOverride);
        lvf::Frame frame;
        if (!source.next(frame)) {
            throw std::runtime_error("no frame returned by KittiDataSource");
        }

        lvf::MappingOptions mappingOptions;
        mappingOptions.x_min = options.xMin;
        mappingOptions.x_max = options.xMax;
        mappingOptions.y_min = options.yMin;
        mappingOptions.y_max = options.yMax;
        mappingOptions.z_min = options.zMin;
        mappingOptions.z_max = options.zMax;
        mappingOptions.voxel_size = options.voxelSize;

        lvf::BevOptions bevOptions;
        bevOptions.x_min = options.xMin;
        bevOptions.x_max = options.xMax;
        bevOptions.y_min = options.yMin;
        bevOptions.y_max = options.yMax;
        bevOptions.resolution = options.resolution;
        bevOptions.raycast = options.raycast;
        bevOptions.inflation_cells = options.inflation;

        const auto start = std::chrono::steady_clock::now();

        lvf::PointCloudPreprocessor preprocessor(mappingOptions);
        const auto preprocessed = preprocessor.process(frame.cloud);

        lvf::BevOccupancyGrid mapper(bevOptions);
        const auto bev = mapper.build(*preprocessed.cloud);

        lvf::OccupancyMapMetadata metadata;
        metadata.frameId = frame.id;
        metadata.heightMin = options.zMin;
        metadata.heightMax = options.zMax;
        metadata.raycast = options.raycast;
        metadata.inflationCells = options.inflation;

        std::filesystem::create_directories(options.outputDir);
        lvf::writeOccupancyMap(options.outputDir, frame.id, bev.grid, metadata);

        lvf::OccupancyRenderOptions renderOptions;
        renderOptions.scale = options.scale;
        renderOptions.draw_grid = options.drawGrid;
        const cv::Mat preview = lvf::renderOccupancyMap(bev.grid, renderOptions);
        lvf::writePng(options.outputDir / (frame.id + "_occupancy.png"), preview);

        const auto end = std::chrono::steady_clock::now();
        const double elapsedMs = std::chrono::duration<double, std::milli>(end - start).count();

        std::cout << "source: " << source.name() << '\n'
                  << "frame: " << frame.id << '\n'
                  << "input points: " << preprocessed.stats.input_points << '\n'
                  << "roi points: " << preprocessed.stats.roi_points << '\n'
                  << "height points: " << preprocessed.stats.height_points << '\n'
                  << "downsampled points: " << preprocessed.stats.downsampled_points << '\n'
                  << "grid: " << bev.grid.width << 'x' << bev.grid.height
                  << " @ " << bev.grid.resolution << " m\n"
                  << "occupied: " << bev.stats.occupied << '\n'
                  << "free: " << bev.stats.free << '\n'
                  << "unknown: " << bev.stats.unknown << '\n'
                  << "raycast: " << (options.raycast ? "true" : "false") << '\n'
                  << "elapsed: " << std::fixed << std::setprecision(2) << elapsedMs << " ms\n"
                  << "output dir: " << options.outputDir.filename().string() << '\n'
                  << "pgm: " << frame.id << "_occupancy.pgm\n"
                  << "yaml: " << frame.id << "_occupancy.yaml\n"
                  << "preview: " << frame.id << "_occupancy.png\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[ERROR] " << error.what() << '\n';
        return 1;
    }
}
