#include "io/OccupancyMapWriter.h"

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

namespace lvf {
namespace {

std::uint8_t pgmValue(std::int8_t cell) {
    if (cell == 100) {
        return 0;  // occupied
    }
    if (cell == 0) {
        return 254;  // free
    }
    return 205;  // unknown
}

}  // namespace

void writeOccupancyMap(const std::filesystem::path& outputDir,
                       const std::string& stem,
                       const GridMap& grid,
                       const OccupancyMapMetadata& metadata) {
    if (grid.width <= 0 || grid.height <= 0) {
        throw std::invalid_argument("writeOccupancyMap: grid dimensions must be positive");
    }
    const std::size_t expected =
        static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height);
    if (grid.cells.size() != expected) {
        throw std::invalid_argument("writeOccupancyMap: cell count does not match dimensions");
    }
    if (grid.resolution <= 0.0f) {
        throw std::invalid_argument("writeOccupancyMap: resolution must be positive");
    }

    std::filesystem::create_directories(outputDir);
    const auto pgmPath = outputDir / (stem + "_occupancy.pgm");
    const auto yamlPath = outputDir / (stem + "_occupancy.yaml");

    {
        std::ofstream pgm(pgmPath, std::ios::binary | std::ios::trunc);
        if (!pgm) {
            throw std::runtime_error("writeOccupancyMap: cannot open " + pgmPath.string());
        }
        pgm << "P5\n" << grid.width << ' ' << grid.height << "\n255\n";
        std::vector<std::uint8_t> row(static_cast<std::size_t>(grid.width));
        for (int y = 0; y < grid.height; ++y) {
            for (int x = 0; x < grid.width; ++x) {
                const std::size_t index =
                    static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) +
                    static_cast<std::size_t>(x);
                row[static_cast<std::size_t>(x)] = pgmValue(grid.cells[index]);
            }
            pgm.write(reinterpret_cast<const char*>(row.data()),
                      static_cast<std::streamsize>(row.size()));
        }
        if (!pgm) {
            throw std::runtime_error("writeOccupancyMap: failed to write " + pgmPath.string());
        }
    }

    {
        std::ofstream yaml(yamlPath, std::ios::trunc);
        if (!yaml) {
            throw std::runtime_error("writeOccupancyMap: cannot open " + yamlPath.string());
        }
        yaml << std::fixed << std::setprecision(4);
        yaml << "image: " << stem << "_occupancy.pgm\n"
             << "resolution: " << grid.resolution << "\n"
             << "origin: [" << grid.origin.x() << ", " << grid.origin.y() << ", 0.0000]\n"
             << "negate: 0\n"
             << "occupied_thresh: 0.65\n"
             << "free_thresh: 0.196\n"
             << "frame_id: velodyne\n"
             << "height_min: " << metadata.heightMin << "\n"
             << "height_max: " << metadata.heightMax << "\n"
             << "raycast: " << (metadata.raycast ? "true" : "false") << "\n"
             << "inflation_cells: " << metadata.inflationCells << "\n";
        if (!yaml) {
            throw std::runtime_error("writeOccupancyMap: failed to write " + yamlPath.string());
        }
    }
}

}  // namespace lvf
