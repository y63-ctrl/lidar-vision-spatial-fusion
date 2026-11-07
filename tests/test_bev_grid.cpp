#include "mapping/BevOccupancyGrid.h"

#include "TestHarness.h"

#include <memory>

namespace {

lvf::CloudXYZI makeCloud(std::initializer_list<pcl::PointXYZI> points) {
    lvf::CloudXYZI cloud;
    for (const auto& point : points) {
        cloud.push_back(point);
    }
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1;
    return cloud;
}

pcl::PointXYZI point(float x, float y) {
    pcl::PointXYZI value;
    value.x = x;
    value.y = y;
    value.z = 0.0f;
    value.intensity = 0.5f;
    return value;
}

}  // namespace

int main() {
    using namespace lvf::test;

    lvf::BevOptions options;
    options.x_min = -5.0f;
    options.x_max = 5.0f;
    options.y_min = -5.0f;
    options.y_max = 5.0f;
    options.resolution = 1.0f;
    options.raycast = true;

    lvf::BevOccupancyGrid mapper(options);

    {
        const auto cloud = makeCloud({point(2.0f, 0.0f)});
        const auto result = mapper.build(cloud);
        check(result.grid.width == 10, "grid width");
        check(result.grid.height == 10, "grid height");
        checkNear(result.grid.origin.x(), -5.0f, 1e-6f, "grid origin x");
        checkNear(result.grid.origin.y(), -5.0f, 1e-6f, "grid origin y");
        check(result.grid.cells.size() == 100, "grid cell count");

        const auto at = [&](int col, int row) {
            return result.grid.cells[static_cast<std::size_t>(row) * 10 +
                                     static_cast<std::size_t>(col)];
        };
        check(at(7, 5) == 100, "point cell is occupied");
        check(at(6, 5) == 0, "ray cell is free");
        check(at(5, 5) == 0, "origin cell is free");
        check(at(0, 0) == -1, "unobserved cell is unknown");
        check(result.stats.occupied == 1, "occupied count");
        check(result.stats.free > 0, "free count");
    }

    {
        const auto cloud = makeCloud({point(0.0f, 2.0f)});
        const auto result = mapper.build(cloud);
        const auto at = [&](int col, int row) {
            return result.grid.cells[static_cast<std::size_t>(row) * 10 +
                                     static_cast<std::size_t>(col)];
        };
        check(at(5, 3) == 100, "positive y is in the upper row");
    }

    {
        const auto cloud = makeCloud({point(1.0f, 0.0f), point(3.0f, 0.0f)});
        const auto result = mapper.build(cloud);
        const auto at = [&](int col, int row) {
            return result.grid.cells[static_cast<std::size_t>(row) * 10 +
                                     static_cast<std::size_t>(col)];
        };
        check(at(6, 5) == 100, "near obstacle occupied");
        check(at(7, 5) == -1, "cell behind obstacle stays unknown");
        check(at(8, 5) == 100, "far point occupied");
    }

    {
        lvf::BevOptions inflated = options;
        inflated.inflation_cells = 1;
        lvf::BevOccupancyGrid inflatedMapper(inflated);
        const auto result = inflatedMapper.build(makeCloud({point(2.0f, 0.0f)}));
        const auto at = [&](int col, int row) {
            return result.grid.cells[static_cast<std::size_t>(row) * 10 +
                                     static_cast<std::size_t>(col)];
        };
        check(at(6, 4) == 100, "inflated neighbor is occupied");
        check(at(8, 6) == 100, "inflated diagonal neighbor is occupied");
    }

    {
        lvf::BevOptions noRaycast = options;
        noRaycast.raycast = false;
        lvf::BevOccupancyGrid plainMapper(noRaycast);
        const auto result = plainMapper.build(makeCloud({point(2.0f, 0.0f)}));
        check(result.stats.free == 0, "no-raycast leaves free at zero");
        check(result.stats.unknown == 99, "no-raycast leaves other cells unknown");
    }

    {
        lvf::BevOptions invalid = options;
        invalid.resolution = 0.0f;
        checkThrows([&] { lvf::BevOccupancyGrid bad(invalid); }, "invalid resolution throws");
    }

    return finish("bev_grid");
}
