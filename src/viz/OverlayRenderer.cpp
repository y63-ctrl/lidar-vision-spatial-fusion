#include "viz/OverlayRenderer.h"

#include "fusion/Projection.h"

#include "../fusion/ColorUtils.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace lvf {
namespace {

void drawLabel(cv::Mat& canvas, const std::string& text, const cv::Point& origin) {
    cv::putText(canvas, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 3,
                cv::LINE_AA);
    cv::putText(canvas, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1,
                cv::LINE_AA);
}

cv::Mat render(const Frame& frame,
               const FusionResult& result,
               const OverlayOptions& options,
               bool withImage) {
    if (frame.image.empty()) {
        throw std::invalid_argument("OverlayRenderer: frame image is empty");
    }
    if (result.cloud == nullptr) {
        throw std::invalid_argument("OverlayRenderer: fusion cloud is null");
    }

    cv::Mat canvas =
        withImage ? frame.image.clone() : cv::Mat::zeros(frame.image.size(), CV_8UC3);
    if (canvas.type() != CV_8UC3) {
        throw std::invalid_argument("OverlayRenderer: expected BGR8 canvas");
    }

    const int width = canvas.cols;
    const int height = canvas.rows;
    std::vector<float> depthBuffer(static_cast<std::size_t>(width) * static_cast<std::size_t>(height),
                                   std::numeric_limits<float>::infinity());

    const ProjectionOptions projectionOptions{
        width, height, options.min_depth, options.max_depth};
    const int pointSize = std::max(1, options.point_size);
    const int offset = pointSize / 2;

    std::vector<Eigen::Vector3f> points;
    points.reserve(result.cloud->size());
    for (const auto& point : result.cloud->points) {
        points.emplace_back(point.x, point.y, point.z);
    }
    const auto projected = Projection::projectBatch(frame.calib, points, projectionOptions, 0);

    for (std::size_t i = 0; i < result.cloud->points.size(); ++i) {
        const auto& projectedPoint = projected[i];
        if (!projectedPoint.valid) {
            continue;
        }

        const int centerX = static_cast<int>(std::floor(projectedPoint.u));
        const int centerY = static_cast<int>(std::floor(projectedPoint.v));
        const auto& point = result.cloud->points[i];
        const cv::Vec3b color =
            withImage ? detail::depthColor(projectedPoint.depth, options.min_depth,
                                           options.max_depth)
                      : cv::Vec3b(point.b, point.g, point.r);

        for (int dy = 0; dy < pointSize; ++dy) {
            for (int dx = 0; dx < pointSize; ++dx) {
                const int x = centerX - offset + dx;
                const int y = centerY - offset + dy;
                if (x < 0 || y < 0 || x >= width || y >= height) {
                    continue;
                }
                const std::size_t index =
                    static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                    static_cast<std::size_t>(x);
                if (projectedPoint.depth < depthBuffer[index]) {
                    depthBuffer[index] = projectedPoint.depth;
                    canvas.at<cv::Vec3b>(y, x) = color;
                }
            }
        }
    }

    if (options.draw_text) {
        const std::string label = "frame " + frame.id + " | colored " +
                                  std::to_string(result.colored_points);
        drawLabel(canvas, label, withImage ? cv::Point(10, height - 10) : cv::Point(10, 24));
    }

    return canvas;
}

}  // namespace

cv::Mat OverlayRenderer::renderOverlay(const Frame& frame,
                                       const FusionResult& result,
                                       const OverlayOptions& options) {
    return render(frame, result, options, true);
}

cv::Mat OverlayRenderer::renderFusedView(const Frame& frame,
                                         const FusionResult& result,
                                         const OverlayOptions& options) {
    return render(frame, result, options, false);
}

}  // namespace lvf
