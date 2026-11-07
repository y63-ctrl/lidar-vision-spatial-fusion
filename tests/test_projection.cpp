#include "fusion/Projection.h"

#include "TestHarness.h"

#include <Eigen/Core>

#include <vector>

namespace {

lvf::Calibration makeCalibration() {
    lvf::Calibration calibration;
    calibration.P2 << 100.0f, 0.0f, 50.0f, 0.0f,
                      0.0f, 100.0f, 40.0f, 0.0f,
                      0.0f, 0.0f, 1.0f, 0.0f;
    calibration.R0_rect = Eigen::Matrix4f::Identity();
    calibration.T_velo_cam = Eigen::Matrix4f::Identity();
    return calibration;
}

void checkPoint(const lvf::ProjectedPoint& point,
                bool valid,
                float u,
                float v,
                const std::string& label) {
    using namespace lvf::test;
    check(point.valid == valid, label + " validity");
    if (valid) {
        checkNear(point.u, u, 1e-4f, label + " u");
        checkNear(point.v, v, 1e-4f, label + " v");
    }
}

}  // namespace

int main() {
    using namespace lvf::test;

    const lvf::Calibration calibration = makeCalibration();
    const lvf::ProjectionOptions options{100, 80, 0.5f, 80.0f};

    const auto center =
        lvf::Projection::projectPoint(calibration, Eigen::Vector3f(0.0f, 0.0f, 10.0f), options);
    checkPoint(center, true, 50.0f, 40.0f, "center");
    checkNear(center.depth, 10.0f, 1e-4f, "center depth");

    checkPoint(lvf::Projection::projectPoint(calibration,
                                             Eigen::Vector3f(0.0f, 0.0f, -1.0f), options),
               false, 0.0f, 0.0f, "behind camera");

    checkPoint(lvf::Projection::projectPoint(calibration,
                                             Eigen::Vector3f(100.0f, 0.0f, 10.0f), options),
               false, 0.0f, 0.0f, "outside image");

    checkPoint(lvf::Projection::projectPoint(calibration,
                                             Eigen::Vector3f(0.0f, 0.0f, 0.1f), options),
               false, 0.0f, 0.0f, "too close");

    checkPoint(lvf::Projection::projectPoint(calibration,
                                             Eigen::Vector3f(0.0f, 0.0f, 81.0f), options),
               false, 0.0f, 0.0f, "too far");

    const std::vector<Eigen::Vector3f> points{
        Eigen::Vector3f(0.0f, 0.0f, 10.0f),
        Eigen::Vector3f(0.0f, 0.0f, -1.0f),
        Eigen::Vector3f(100.0f, 0.0f, 10.0f),
        Eigen::Vector3f(0.0f, 0.0f, 20.0f),
    };

    const auto singleThread = lvf::Projection::projectBatch(calibration, points, options, 1);
    const auto fourThreads = lvf::Projection::projectBatch(calibration, points, options, 4);
    check(singleThread.size() == points.size(), "batch size (single thread)");
    check(fourThreads.size() == points.size(), "batch size (four threads)");

    int validCount = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (singleThread[i].valid) {
            ++validCount;
        }
        check(singleThread[i].valid == fourThreads[i].valid, "thread-independent validity");
        checkNear(singleThread[i].u, fourThreads[i].u, 1e-6f, "thread-independent u");
        checkNear(singleThread[i].v, fourThreads[i].v, 1e-6f, "thread-independent v");
        checkNear(singleThread[i].depth, fourThreads[i].depth, 1e-6f, "thread-independent depth");
    }
    check(validCount == 2, "batch valid count");

    checkThrows([] {
        const lvf::ProjectionOptions invalid{0, 80, 0.5f, 80.0f};
        (void)lvf::Projection::projectPoint(makeCalibration(), Eigen::Vector3f::Zero(), invalid);
    }, "zero image width throws");

    checkThrows([] {
        const lvf::ProjectionOptions invalid{100, 80, 10.0f, 10.0f};
        (void)lvf::Projection::projectPoint(makeCalibration(), Eigen::Vector3f::Zero(), invalid);
    }, "invalid depth range throws");

    return finish("projection");
}
