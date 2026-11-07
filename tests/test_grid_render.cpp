#include "viz/OccupancyMapRenderer.h"

#include "TestHarness.h"

#include <opencv2/core.hpp>

namespace {

bool sameColor(const cv::Vec3b& a, const cv::Vec3b& b) {
    return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

}  // namespace

int main() {
    using namespace lvf::test;

    lvf::GridMap grid;
    grid.width = 4;
    grid.height = 3;
    grid.resolution = 0.5f;
    grid.origin = Eigen::Vector2f(-1.0f, -1.0f);
    grid.cells = {
        0, 100, -1, 0,
        100, -1, 0, 100,
        -1, 0, 100, -1,
    };

    lvf::OccupancyRenderOptions options;
    options.scale = 2;
    options.draw_grid = false;
    const cv::Mat image = lvf::renderOccupancyMap(grid, options);
    check(image.cols == 8, "render width");
    check(image.rows == 6, "render height");
    check(image.type() == CV_8UC3, "render type");
    check(sameColor(image.at<cv::Vec3b>(0, 0), cv::Vec3b(255, 255, 255)), "free is white");
    check(sameColor(image.at<cv::Vec3b>(0, 2), cv::Vec3b(0, 0, 0)), "occupied is black");
    check(sameColor(image.at<cv::Vec3b>(0, 4), cv::Vec3b(205, 205, 205)), "unknown is gray");

    lvf::GridMap broken = grid;
    broken.cells.pop_back();
    checkThrows([&] { (void)lvf::renderOccupancyMap(broken, options); },
                "cell count mismatch throws");
    options.scale = 0;
    checkThrows([&] { (void)lvf::renderOccupancyMap(grid, options); }, "zero scale throws");

    return finish("grid_render");
}
