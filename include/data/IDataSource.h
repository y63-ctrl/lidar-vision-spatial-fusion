#pragma once

#include "core/Types.h"

#include <string>

namespace lvf {

/// Abstract frame source. Implementations may read from disk, a dataset or a
/// test fixture; the pipeline only depends on this interface.
///
/// Contract:
///   - next(frame) returns false when no frames remain.
///   - On success, `frame` is fully populated and replaces any previous content.
///   - On I/O or parse failure, implementations throw std::runtime_error with a
///     message that contains the offending path.
///   - A source instance is not thread-safe.
class IDataSource {
public:
    virtual ~IDataSource() = default;

    virtual bool next(Frame& frame) = 0;
    virtual std::string name() const = 0;
};

}  // namespace lvf
