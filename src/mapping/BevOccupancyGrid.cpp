#include "mapping/BevOccupancyGrid.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lvf {
namespace {

void validateOptions(const BevOptions& options) {
    if (!(options.x_min < options.x_max) || !(options.y_min < options.y_max)) {
        throw std::invalid_argument("BevOccupancyGrid: invalid ROI bounds");
    }
    if (options.resolution <= 0.0f) {
        throw std::invalid_argument("BevOccupancyGrid: resolution must be positive");
    }
    if (options.inflation_cells < 0) {
        throw std::invalid_argument("BevOccupancyGrid: inflation_cells must be non-negative");
    }
}

int cellIndex(int col, int row, int width) {
    return row * width + col;
}

void castRay(int originCol,
             int originRow,
             int targetCol,
             int targetRow,
             std::vector<std::int8_t>& cells,
             int width,
             int height) {
    int x = originCol;
    int y = originRow;
    const int dx = std::abs(targetCol - originCol);
    const int sx = originCol < targetCol ? 1 : -1;
    const int dy = -std::abs(targetRow - originRow);
    const int sy = originRow < targetRow ? 1 : -1;
    int error = dx + dy;

    while (x != targetCol || y != targetRow) {
        const int e2 = 2 * error;
        if (e2 >= dy) {
            error += dy;
            x += sx;
        }
        if (e2 <= dx) {
            error += dx;
            y += sy;
        }

        if (x == targetCol && y == targetRow) {
            break;
        }
        if (x < 0 || y < 0 || x >= width || y >= height) {
            continue;
        }

        auto& cell = cells[static_cast<std::size_t>(cellIndex(x, y, width))];
        if (cell == 100) {
            break;  // an obstacle blocks the ray before the target
        }
        if (cell == -1) {
            cell = 0;
        }
    }
}

}  // namespace

BevOccupancyGrid::BevOccupancyGrid(BevOptions options) : options_(options) {
    validateOptions(options_);
}

BevResult BevOccupancyGrid::build(const CloudXYZI& cloud) const {
    const int width =
        std::max(1, static_cast<int>(std::ceil((options_.x_max - options_.x_min) /
                                               options_.resolution)));
    const int height =
        std::max(1, static_cast<int>(std::ceil((options_.y_max - options_.y_min) /
                                               options_.resolution)));
    const std::size_t total = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);

    GridMap grid;
    grid.width = width;
    grid.height = height;
    grid.resolution = options_.resolution;
    grid.origin = Eigen::Vector2f(options_.x_min, options_.y_min);
    grid.cells.assign(total, -1);

    for (const auto& point : cloud.points) {
        if (point.x < options_.x_min || point.x >= options_.x_max ||
            point.y < options_.y_min || point.y >= options_.y_max) {
            continue;
        }
        const int col = static_cast<int>(std::floor(
            (point.x - options_.x_min) / options_.resolution));
        const int row = static_cast<int>(std::floor(
            (options_.y_max - point.y) / options_.resolution));
        if (col < 0 || row < 0 || col >= width || row >= height) {
            continue;
        }
        grid.cells[static_cast<std::size_t>(cellIndex(col, row, width))] = 100;
    }

    const int originCol = static_cast<int>(std::floor(
        (0.0f - options_.x_min) / options_.resolution));
    const int originRow = static_cast<int>(std::floor(
        (options_.y_max - 0.0f) / options_.resolution));

    if (options_.raycast) {
        if (originCol < 0 || originRow < 0 || originCol >= width || originRow >= height) {
            throw std::invalid_argument("BevOccupancyGrid: sensor origin is outside the BEV ROI");
        }

        const std::size_t originIndex =
            static_cast<std::size_t>(cellIndex(originCol, originRow, width));
        if (grid.cells[originIndex] == -1) {
            grid.cells[originIndex] = 0;
        }

        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                if (grid.cells[static_cast<std::size_t>(cellIndex(col, row, width))] != 100) {
                    continue;
                }
                castRay(originCol, originRow, col, row, grid.cells, width, height);
            }
        }
    }

    if (options_.inflation_cells > 0) {
        const std::vector<std::int8_t> before = grid.cells;
        const int radius = options_.inflation_cells;
        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                if (before[static_cast<std::size_t>(cellIndex(col, row, width))] != 100) {
                    continue;
                }
                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dx = -radius; dx <= radius; ++dx) {
                        const int x = col + dx;
                        const int y = row + dy;
                        if (x < 0 || y < 0 || x >= width || y >= height) {
                            continue;
                        }
                        grid.cells[static_cast<std::size_t>(cellIndex(x, y, width))] = 100;
                    }
                }
            }
        }
    }

    BevStats stats;
    stats.total = total;
    for (const auto value : grid.cells) {
        if (value == 100) {
            ++stats.occupied;
        } else if (value == 0) {
            ++stats.free;
        } else {
            ++stats.unknown;
        }
    }

    return BevResult{std::move(grid), stats};
}

}  // namespace lvf
