#pragma once

#include "core/Types.h"

namespace lvf {

/// Strategy interface for early fusion. Implemented in M2 by:
///   - PinholeFusionStrategy: RGB sampling from the camera image
///   - DepthOnlyFusionStrategy: pseudo-color by depth when the image is degraded
class IFusionStrategy {
public:
    virtual ~IFusionStrategy() = default;

    virtual FusionResult fuse(const Frame& frame) const = 0;
};

}  // namespace lvf
