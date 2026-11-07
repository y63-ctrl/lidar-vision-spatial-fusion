#include "viz/OccupancyMapRenderer.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace lvf {

cv::Mat renderOccupancyMap(const GridMap& grid, const OccupancyRenderOptions& options) {
    if (grid.width <= 0 || grid.height <= 0) {
        throw std::invalid_argument("renderOccupancyMap: grid dimensions must be positive");
    }
    if (grid.cells.size() !=
        static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height)) {
        throw std::invalid_argument("renderOccupancyMap: cell count does not match dimensions");
    }
    if (options.scale <= 0) {
        throw std::invalid_argument("renderOccupancyMap: scale must be positive");
    }

    cv::Mat small(grid.height, grid.width, CV_8UC3);
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            const std::size_t index =
                static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) +
                static_cast<std::size_t>(x);
            const auto value = grid.cells[index];
            if (value == 100) {
                small.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
            } else if (value == 0) {
                small.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
            } else {
                small.at<cv::Vec3b>(y, x) = cv::Vec3b(205, 205, 205);
            }
        }
    }

    cv::Mat output;
    if (options.scale == 1) {
        output = small;
    } else {
        cv::resize(small, output, cv::Size(grid.width * options.scale, grid.height * options.scale),
                   0.0, 0.0, cv::INTER_NEAREST);
    }

    if (options.draw_grid && grid.resolution > 0.0f) {
        const int stepPixels =
            std::max(1, static_cast<int>(std::lround(10.0f / grid.resolution)) * options.scale);
        const cv::Scalar lineColor(160, 160, 160);
        for (int x = 0; x < output.cols; x += stepPixels) {
            cv::line(output, cv::Point(x, 0), cv::Point(x, output.rows - 1), lineColor, 1,
                     cv::LINE_AA);
        }
        for (int y = 0; y < output.rows; y += stepPixels) {
            cv::line(output, cv::Point(0, y), cv::Point(output.cols - 1, y), lineColor, 1,
                     cv::LINE_AA);
        }
    }

    return output;
}

}  // namespace lvf
