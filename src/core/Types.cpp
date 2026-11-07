#include "core/Types.h"

#include <cmath>

namespace lvf {

Eigen::Matrix<float, 3, 4> Calibration::veloToImage() const {
    return P2 * R0_rect * T_velo_cam;
}

Eigen::Vector3f Calibration::veloToCamera(const Eigen::Vector3f& veloPoint) const {
    const Eigen::Vector4f homogeneous(veloPoint.x(), veloPoint.y(), veloPoint.z(), 1.0f);
    return (T_velo_cam * homogeneous).head<3>();
}

bool Calibration::isValid() const {
    if (!P2.allFinite() || !R0_rect.allFinite() || !T_velo_cam.allFinite()) {
        return false;
    }

    // P2's third row should be approximately [0, 0, 1, small].
    if (std::abs(P2(2, 0)) > 1e-3f || std::abs(P2(2, 1)) > 1e-3f ||
        std::abs(P2(2, 2) - 1.0f) > 1e-3f) {
        return false;
    }

    // Bottom row of homogeneous transforms must be [0, 0, 0, 1].
    for (int col = 0; col < 3; ++col) {
        if (std::abs(R0_rect(3, col)) > 1e-6f ||
            std::abs(T_velo_cam(3, col)) > 1e-6f) {
            return false;
        }
    }
    if (std::abs(R0_rect(3, 3) - 1.0f) > 1e-6f ||
        std::abs(T_velo_cam(3, 3) - 1.0f) > 1e-6f) {
        return false;
    }

    return true;
}

}  // namespace lvf
