#include "io/OccupancyMapWriter.h"

#include "TestHarness.h"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string readText(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

}  // namespace

int main() {
    using namespace lvf::test;

    const auto directory = std::filesystem::temp_directory_path() / "lvf_map_io";
    std::filesystem::remove_all(directory);

    lvf::GridMap grid;
    grid.width = 4;
    grid.height = 3;
    grid.resolution = 0.5f;
    grid.origin = Eigen::Vector2f(-1.0f, -1.0f);
    grid.cells = {
        100, 0, -1, 100,
        0, -1, 100, 0,
        -1, 0, 100, 100,
    };

    lvf::OccupancyMapMetadata metadata;
    metadata.frameId = "000000";
    metadata.heightMin = -1.5f;
    metadata.heightMax = 2.5f;
    metadata.raycast = true;

    lvf::writeOccupancyMap(directory, "test", grid, metadata);
    const auto pgmPath = directory / "test_occupancy.pgm";
    const auto yamlPath = directory / "test_occupancy.yaml";
    check(std::filesystem::exists(pgmPath), "PGM exists");
    check(std::filesystem::exists(yamlPath), "YAML exists");

    {
        std::ifstream pgm(pgmPath, std::ios::binary);
        std::string magic;
        int width = 0;
        int height = 0;
        int maxValue = 0;
        pgm >> magic >> width >> height >> maxValue;
        pgm.get();
        check(magic == "P5", "PGM magic");
        check(width == 4 && height == 3, "PGM dimensions");
        check(maxValue == 255, "PGM max value");

        std::vector<std::uint8_t> pixels(12);
        pgm.read(reinterpret_cast<char*>(pixels.data()),
                 static_cast<std::streamsize>(pixels.size()));
        check(pixels[0] == 0, "occupied pixel");
        check(pixels[1] == 254, "free pixel");
        check(pixels[2] == 205, "unknown pixel");
        check(pixels[11] == 0, "last occupied pixel");
    }

    const std::string yaml = readText(yamlPath);
    check(yaml.find("image: test_occupancy.pgm") != std::string::npos, "YAML image");
    check(yaml.find("resolution: 0.5000") != std::string::npos, "YAML resolution");
    check(yaml.find("origin: [-1.0000, -1.0000, 0.0000]") != std::string::npos, "YAML origin");
    check(yaml.find("frame_id: velodyne") != std::string::npos, "YAML frame_id");
    check(yaml.find("occupied_thresh: 0.65") != std::string::npos, "YAML occupied threshold");
    check(yaml.find("free_thresh: 0.196") != std::string::npos, "YAML free threshold");

    lvf::GridMap broken = grid;
    broken.cells.pop_back();
    checkThrows([&] { lvf::writeOccupancyMap(directory, "broken", broken, metadata); },
                "cell count mismatch throws");

    std::filesystem::remove_all(directory);
    return finish("map_io");
}
