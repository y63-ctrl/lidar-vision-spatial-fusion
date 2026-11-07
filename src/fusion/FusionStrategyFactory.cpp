#include "fusion/FusionStrategyFactory.h"

#include "fusion/DepthOnlyFusionStrategy.h"
#include "fusion/PinholeFusionStrategy.h"

#include <stdexcept>

namespace lvf {

std::unique_ptr<IFusionStrategy> FusionStrategyFactory::create(FusionMode mode,
                                                               const FusionOptions& options) {
    switch (mode) {
        case FusionMode::Pinhole:
            return std::make_unique<PinholeFusionStrategy>(options);
        case FusionMode::DepthOnly:
            return std::make_unique<DepthOnlyFusionStrategy>(options);
    }
    throw std::invalid_argument("FusionStrategyFactory: unknown FusionMode");
}

std::unique_ptr<IFusionStrategy> FusionStrategyFactory::create(const std::string& name,
                                                               const FusionOptions& options) {
    if (name == "pinhole" || name == "rgb" || name == "Pinhole") {
        return create(FusionMode::Pinhole, options);
    }
    if (name == "depth" || name == "depth_only" || name == "DepthOnly") {
        return create(FusionMode::DepthOnly, options);
    }
    throw std::invalid_argument("FusionStrategyFactory: unknown strategy '" + name + "'");
}

std::unique_ptr<IFusionStrategy> FusionStrategyFactory::createForAction(
    const std::string& action, const FusionOptions& options) {
    if (action == "FULL_FUSION") {
        return create(FusionMode::Pinhole, options);
    }
    if (action == "GEOMETRY_ONLY" || action == "IMAGE_ONLY" || action == "BOTH_DEGRADED") {
        // Even when the image is degraded, geometry (LiDAR depth) can still be
        // rendered. M3 will refine the action set and map trust flag.
        return create(FusionMode::DepthOnly, options);
    }
    throw std::invalid_argument("FusionStrategyFactory: unknown degradation action '" + action +
                                "'");
}

}  // namespace lvf
