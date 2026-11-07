#include "core/Version.h"

#include <Eigen/Core>
#include <opencv2/core/version.hpp>
#include <pcl/pcl_config.h>

#include <sstream>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace lvf {

std::string version() {
    return "0.1.0";
}

std::string dependencySummary() {
    std::ostringstream out;
    out << "OpenCV " << CV_VERSION
        << " | Eigen " << EIGEN_WORLD_VERSION << '.' << EIGEN_MAJOR_VERSION << '.'
        << EIGEN_MINOR_VERSION
        << " | PCL " << PCL_MAJOR_VERSION << '.' << PCL_MINOR_VERSION << '.'
        << PCL_REVISION_VERSION;
    return out.str();
}

bool hasOpenMp() {
#ifdef _OPENMP
    return true;
#else
    return false;
#endif
}

}  // namespace lvf
