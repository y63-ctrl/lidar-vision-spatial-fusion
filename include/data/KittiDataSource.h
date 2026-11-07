#pragma once

#include "core/Types.h"
#include "data/IDataSource.h"

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace lvf {

/// KITTI object-benchmark data source.
///
/// Expected layout:
///   <root>/image_2/<frame>.png
///   <root>/velodyne/<frame>.bin
///   <root>/calib/<frame>.txt
///
/// `calibOverride` replaces the per-frame calib file and is useful for
/// demonstrating extrinsic drift.
class KittiDataSource final : public IDataSource {
public:
    explicit KittiDataSource(std::filesystem::path dataRoot,
                             std::vector<std::string> frameIds = {},
                             std::filesystem::path calibOverride = {});

    bool next(Frame& frame) override;
    std::string name() const override;

    /// Discover frame IDs from image_2/*.png, sorted lexicographically.
    static std::vector<std::string> discoverFrames(const std::filesystem::path& dataRoot);

    /// Normalize "0" / "000000" / "000000.png" to "000000".
    static std::string normalizeFrameId(const std::string& frameId);

private:
    std::filesystem::path imagePath(const std::string& frameId) const;
    std::filesystem::path cloudPath(const std::string& frameId) const;
    std::filesystem::path calibPath(const std::string& frameId) const;

    std::filesystem::path dataRoot_;
    std::filesystem::path calibOverride_;
    std::vector<std::string> frameIds_;
    std::size_t nextIndex_ = 0;
};

}  // namespace lvf
