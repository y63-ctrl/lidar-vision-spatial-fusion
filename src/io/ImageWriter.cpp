#include "io/ImageWriter.h"

#include <opencv2/imgcodecs.hpp>

#include <fstream>
#include <stdexcept>
#include <vector>

namespace lvf {

void writePng(const std::filesystem::path& path, const cv::Mat& image) {
    if (image.empty()) {
        throw std::invalid_argument("writePng: image is empty");
    }

    std::vector<unsigned char> encoded;
    if (!cv::imencode(".png", image, encoded) || encoded.empty()) {
        throw std::runtime_error("writePng: PNG encoding failed");
    }

    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }

    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output) {
        throw std::runtime_error("writePng: cannot open " + path.string());
    }
    output.write(reinterpret_cast<const char*>(encoded.data()),
                 static_cast<std::streamsize>(encoded.size()));
    if (!output) {
        throw std::runtime_error("writePng: failed to write " + path.string());
    }
}

}  // namespace lvf
