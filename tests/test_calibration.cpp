#include "data/KittiCalibration.h"

#include "TestHarness.h"

#include <Eigen/Core>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

const char* kCalibrationText = R"(
P0: 100 0 50 0 0 100 40 0 0 0 1 0
P1: 100 0 50 0 0 100 40 0 0 0 1 0
P2: 100 0 50 0
     0 100 40 0
     0 0 1 0
P3: 100 0 50 0 0 100 40 0 0 0 1 0
R0_rect: 1 0 0
         0 1 0
         0 0 1
Tr_velo_to_cam: 0 -1 0 0
                0 0 -1 0
                1 0 0 0
Tr_imu_to_velo: 1 0 0 0 0 1 0 0 0 0 1 0
)";

lvf::Calibration parseText(const std::string& text) {
    std::istringstream input(text);
    return lvf::KittiCalibration::parse(input, "test");
}

}  // namespace

int main() {
    using namespace lvf::test;

    const lvf::Calibration calibration = parseText(kCalibrationText);
    check(calibration.isValid(), "parsed calibration is valid");
    checkNear(calibration.P2(0, 0), 100.0f, 1e-4f, "P2 fx");
    checkNear(calibration.P2(0, 2), 50.0f, 1e-4f, "P2 cx");
    checkNear(calibration.P2(1, 2), 40.0f, 1e-4f, "P2 cy");
    checkNear(calibration.R0_rect(0, 0), 1.0f, 1e-6f, "R0_rect identity");

    const Eigen::Vector3f camera = calibration.veloToCamera(Eigen::Vector3f(10.0f, 2.0f, 1.0f));
    checkNear(camera.x(), -2.0f, 1e-5f, "Velodyne y -> camera x");
    checkNear(camera.y(), -1.0f, 1e-5f, "Velodyne z -> camera y");
    checkNear(camera.z(), 10.0f, 1e-5f, "Velodyne x -> camera z");

    const auto projection = calibration.veloToImage();
    const Eigen::Vector4f point(10.0f, 2.0f, 1.0f, 1.0f);
    const Eigen::Vector3f uvw = projection * point;
    checkNear(uvw.x() / uvw.z(), 30.0f, 1e-5f, "projected u");
    checkNear(uvw.y() / uvw.z(), 30.0f, 1e-5f, "projected v");
    checkNear(uvw.z(), 10.0f, 1e-5f, "projected depth");

    {
        const std::string smallP2Offset =
            "P2: 100 0 50 0 0 100 40 0 0 0 1 0.0027\n"
            "R0_rect: 1 0 0 0 1 0 0 0 1\n"
            "Tr_velo_to_cam: 0 -1 0 0 0 0 -1 0 1 0 0 0\n";
        const auto withOffset = parseText(smallP2Offset);
        check(withOffset.isValid(), "KITTI-like small P2(2,3) offset is accepted");
        checkNear(withOffset.P2(2, 3), 0.0027f, 1e-6f, "P2(2,3) is preserved");
    }

    {
        const std::string missingP2 =
            "R0_rect: 1 0 0 0 1 0 0 0 1\n"
            "Tr_velo_to_cam: 0 -1 0 0 0 0 -1 0 1 0 0 0\n";
        checkThrows([&] { parseText(missingP2); }, "missing P2 throws");
    }

    {
        const std::string shortP2 =
            "P2: 100 0 50 0 0 100 40 0 0 0 1\n"
            "R0_rect: 1 0 0 0 1 0 0 0 1\n"
            "Tr_velo_to_cam: 0 -1 0 0 0 0 -1 0 1 0 0 0\n";
        checkThrows([&] { parseText(shortP2); }, "P2 with wrong value count throws");
    }

    {
        const std::string badLastRow =
            "P2: 100 0 50 0 0 100 40 0 1 2 3 0\n"
            "R0_rect: 1 0 0 0 1 0 0 0 1\n"
            "Tr_velo_to_cam: 0 -1 0 0 0 0 -1 0 1 0 0 0\n";
        checkThrows([&] { parseText(badLastRow); }, "P2 with bad last row throws");
    }

    {
        const std::string missingTransform =
            "P2: 100 0 50 0 0 100 40 0 0 0 1 0\n"
            "R0_rect: 1 0 0 0 1 0 0 0 1\n";
        checkThrows([&] { parseText(missingTransform); }, "missing Tr_velo_to_cam throws");
    }

    {
        const auto tempPath =
            std::filesystem::temp_directory_path() / "lvf_test_calibration.txt";
        {
            std::ofstream output(tempPath);
            output << kCalibrationText;
        }
        const auto fromFile = lvf::KittiCalibration::loadFromFile(tempPath);
        check(fromFile.isValid(), "loadFromFile returns a valid calibration");
        std::error_code ignored;
        std::filesystem::remove(tempPath, ignored);
    }

    return finish("calibration");
}
