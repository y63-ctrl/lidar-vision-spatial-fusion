#include "data/KittiDataSource.h"

#include "data/KittiCalibration.h"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace lvf {
namespace {

[[noreturn]] void dataError(const std::filesystem::path& path, const std::string& detail) {
    throw std::runtime_error("KittiDataSource(" + path.string() + "): " + detail);
}

bool hasPngExtension(const std::filesystem::path& path) {
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return extension == ".png";
}

cv::Mat loadImage(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input) {
        dataError(path, "cannot open image");
    }

    const auto end = input.tellg();
    const auto size = static_cast<std::streamoff>(end);
    if (size <= 0) {
        dataError(path, "image file is empty");
    }

    std::vector<unsigned char> bytes(static_cast<std::size_t>(size));
    input.seekg(0, std::ios::beg);
    input.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    if (!input || input.gcount() != static_cast<std::streamsize>(bytes.size())) {
        dataError(path, "failed to read image bytes");
    }

    cv::Mat image = cv::imdecode(bytes, cv::IMREAD_COLOR);
    if (image.empty()) {
        dataError(path, "OpenCV failed to decode image");
    }
    return image;
}

CloudXYZI::Ptr loadCloud(const std::filesystem::path& path, const std::string& frameId) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input) {
        dataError(path, "cannot open point cloud for frame " + frameId);
    }

    const auto end = input.tellg();
    const auto bytes = static_cast<std::streamoff>(end);
    if (bytes <= 0) {
        dataError(path, "point cloud file is empty");
    }
    if (bytes % 16 != 0) {
        std::ostringstream message;
        message << "point cloud size must be a multiple of 16 bytes (x,y,z,intensity); got "
                << bytes;
        dataError(path, message.str());
    }

    const auto pointCount = static_cast<std::size_t>(bytes / 16);
    if (pointCount > std::numeric_limits<std::uint32_t>::max()) {
        dataError(path, "point cloud is too large for PCL's uint32 width");
    }

    std::vector<float> raw(pointCount * 4);
    input.seekg(0, std::ios::beg);
    input.read(reinterpret_cast<char*>(raw.data()), static_cast<std::streamsize>(bytes));
    if (!input || input.gcount() != static_cast<std::streamsize>(bytes)) {
        dataError(path, "failed to read point cloud bytes");
    }

    auto cloud = std::make_shared<CloudXYZI>();
    cloud->points.resize(pointCount);
    for (std::size_t i = 0; i < pointCount; ++i) {
        auto& point = cloud->points[i];
        point.x = raw[i * 4 + 0];
        point.y = raw[i * 4 + 1];
        point.z = raw[i * 4 + 2];
        point.intensity = raw[i * 4 + 3];

        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
            std::ostringstream message;
            message << "non-finite point at index " << i;
            dataError(path, message.str());
        }
    }

    cloud->width = static_cast<std::uint32_t>(pointCount);
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

}  // namespace

KittiDataSource::KittiDataSource(std::filesystem::path dataRoot,
                                 std::vector<std::string> frameIds,
                                 std::filesystem::path calibOverride)
    : dataRoot_(std::move(dataRoot)), calibOverride_(std::move(calibOverride)) {
    if (!std::filesystem::exists(dataRoot_)) {
        dataError(dataRoot_, "data root does not exist");
    }
    if (!std::filesystem::is_directory(dataRoot_)) {
        dataError(dataRoot_, "data root is not a directory");
    }

    if (frameIds.empty()) {
        frameIds_ = discoverFrames(dataRoot_);
    } else {
        frameIds_.reserve(frameIds.size());
        for (const auto& id : frameIds) {
            frameIds_.push_back(normalizeFrameId(id));
        }
    }

    if (frameIds_.empty()) {
        dataError(dataRoot_, "no frames specified or discovered");
    }
}

bool KittiDataSource::next(Frame& frame) {
    if (nextIndex_ >= frameIds_.size()) {
        return false;
    }

    const std::string& frameId = frameIds_[nextIndex_];

    Frame loaded;
    loaded.id = frameId;
    loaded.image = loadImage(imagePath(frameId));
    loaded.cloud = loadCloud(cloudPath(frameId), frameId);
    loaded.calib = KittiCalibration::loadFromFile(calibPath(frameId));

    if (!loaded.calib.isValid()) {
        dataError(calibPath(frameId), "calibration failed validation for frame " + frameId);
    }

    frame = std::move(loaded);
    ++nextIndex_;
    return true;
}

std::string KittiDataSource::name() const {
    return "KittiDataSource";
}

std::vector<std::string> KittiDataSource::discoverFrames(const std::filesystem::path& dataRoot) {
    const auto imageDir = dataRoot / "image_2";
    if (!std::filesystem::exists(imageDir)) {
        dataError(imageDir, "image_2 directory does not exist");
    }
    if (!std::filesystem::is_directory(imageDir)) {
        dataError(imageDir, "image_2 is not a directory");
    }

    std::vector<std::string> frameIds;
    for (const auto& entry : std::filesystem::directory_iterator(imageDir)) {
        if (!entry.is_regular_file() || !hasPngExtension(entry.path())) {
            continue;
        }
        frameIds.push_back(entry.path().stem().string());
    }

    std::sort(frameIds.begin(), frameIds.end());
    if (frameIds.empty()) {
        dataError(imageDir, "no .png frames found");
    }
    return frameIds;
}

std::string KittiDataSource::normalizeFrameId(const std::string& frameId) {
    if (frameId.empty()) {
        throw std::invalid_argument("KittiDataSource: empty frame id");
    }

    std::string id = frameId;
    if (id.size() > 4) {
        const std::string extension = id.substr(id.size() - 4);
        if (extension == ".png" || extension == ".PNG" || extension == ".Png") {
            id.resize(id.size() - 4);
        }
    }

    if (id.find_first_of("/\\") != std::string::npos) {
        throw std::invalid_argument("KittiDataSource: frame id must not contain a path separator: " +
                                    frameId);
    }
    if (id.empty()) {
        throw std::invalid_argument("KittiDataSource: empty frame id after removing extension");
    }

    const bool allDigits = std::all_of(id.begin(), id.end(), [](unsigned char c) {
        return std::isdigit(c) != 0;
    });
    if (allDigits) {
        if (id.size() > 6) {
            throw std::invalid_argument("KittiDataSource: KITTI frame id has more than 6 digits: " +
                                        frameId);
        }
        if (id.size() < 6) {
            id.insert(0, 6 - id.size(), '0');
        }
    }

    return id;
}

std::filesystem::path KittiDataSource::imagePath(const std::string& frameId) const {
    return dataRoot_ / "image_2" / (frameId + ".png");
}

std::filesystem::path KittiDataSource::cloudPath(const std::string& frameId) const {
    return dataRoot_ / "velodyne" / (frameId + ".bin");
}

std::filesystem::path KittiDataSource::calibPath(const std::string& frameId) const {
    if (!calibOverride_.empty()) {
        return calibOverride_;
    }
    return dataRoot_ / "calib" / (frameId + ".txt");
}

}  // namespace lvf
