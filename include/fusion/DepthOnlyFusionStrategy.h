#pragma once

#include "core/Types.h"
#include "fusion/FusionOptions.h"
#include "fusion/IFusionStrategy.h"

namespace lvf {

class DepthOnlyFusionStrategy final : public IFusionStrategy {
public:
    explicit DepthOnlyFusionStrategy(FusionOptions options = {});

    FusionResult fuse(const Frame& frame) const override;

private:
    FusionOptions options_;
};

}  // namespace lvf
