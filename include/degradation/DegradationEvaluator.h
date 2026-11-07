#pragma once

#include "core/Types.h"
#include "degradation/ImageDegradationCheck.h"
#include "degradation/LidarDegradationCheck.h"

namespace lvf {

struct DegradationOptions {
    ImageDegradationOptions image;
    LidarDegradationOptions lidar;
    float image_weight = 0.5f;
};

/// Runs image and LiDAR checks, assigns a combined confidence and a fusion
/// action. The action is consumed by FusionStrategyFactory::createForAction().
class DegradationEvaluator {
public:
    explicit DegradationEvaluator(DegradationOptions options = {});

    DegradationReport evaluate(const Frame& frame) const;

private:
    DegradationOptions options_;
};

}  // namespace lvf
