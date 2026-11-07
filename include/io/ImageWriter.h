#pragma once

#include <opencv2/core.hpp>

#include <filesystem>

namespace lvf {

/// Write a PNG through an in-memory encode, so non-ASCII Windows paths work.
void writePng(const std::filesystem::path& path, const cv::Mat& image);

}  // namespace lvf
