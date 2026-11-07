#pragma once

#include "core/Types.h"
#include "fusion/FusionOptions.h"
#include "fusion/IFusionStrategy.h"

#include <memory>
#include <string>

namespace lvf {

enum class FusionMode {
    Pinhole,
    DepthOnly,
};

class FusionStrategyFactory {
public:
    static std::unique_ptr<IFusionStrategy> create(FusionMode mode,
                                                   const FusionOptions& options = {});
    static std::unique_ptr<IFusionStrategy> create(const std::string& name,
                                                   const FusionOptions& options = {});
    static std::unique_ptr<IFusionStrategy> createForAction(const std::string& action,
                                                            const FusionOptions& options = {});
};

}  // namespace lvf
