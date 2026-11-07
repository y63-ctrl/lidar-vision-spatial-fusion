#pragma once

#include "core/Types.h"
#include "fusion/FusionOptions.h"
#include "fusion/IFusionStrategy.h"

namespace lvf {

class PinholeFusionStrategy final : public IFusionStrategy {
public:
    explicit PinholeFusionStrategy(FusionOptions options = {});

    FusionResult fuse(const Frame& frame) const override;

private:
    FusionOptions options_;
};

}  // namespace lvf
