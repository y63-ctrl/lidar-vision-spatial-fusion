cmake_minimum_required(VERSION 3.20)

project(LiDARVisionFusion
    VERSION 0.1.0
    DESCRIPTION "LiDAR-Vision Spatial Fusion Pipeline"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LVF_BUILD_TESTS "Build unit and smoke tests" ON)
option(LVF_BUILD_TOOLS "Build developer tools" ON)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters)
find_package(OpenMP)

message(STATUS "OpenCV : ${OpenCV_VERSION}")
message(STATUS "Eigen  : ${Eigen3_VERSION}")
message(STATUS "PCL    : ${PCL_VERSION}")
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP : ${OpenMP_CXX_VERSION} (${OpenMP_CXX_FLAGS})")
else()
    message(WARNING "OpenMP was not found; projection will be single-threaded.")
endif()

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
function(lvf_enable_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /Zc:__cplusplus)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

# ---------------------------------------------------------------------------
# Core library
# ---------------------------------------------------------------------------
add_library(lvf_core STATIC
    src/core/Types.cpp
    src/core/Version.cpp
    src/data/KittiCalibration.cpp
    src/data/KittiDataSource.cpp
    src/degradation/DegradationEvaluator.cpp
    src/degradation/ImageDegradationCheck.cpp
    src/degradation/LidarDegradationCheck.cpp
    src/fusion/DepthOnlyFusionStrategy.cpp
    src/fusion/FusionStrategyFactory.cpp
    src/fusion/PinholeFusionStrategy.cpp
    src/fusion/Projection.cpp
    src/io/DegradationJsonWriter.cpp
    src/io/ImageWriter.cpp
    src/io/OccupancyMapWriter.cpp
    src/io/PcdWriter.cpp
    src/io/TimingJsonWriter.cpp
    src/mapping/BevOccupancyGrid.cpp
    src/mapping/PointCloudPreprocessor.cpp
    src/pipeline/Pipeline.cpp
    src/utils/Log.cpp
    src/viz/OccupancyMapRenderer.cpp
    src/viz/OverlayRenderer.cpp
)

target_include_directories(lvf_core
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
    SYSTEM PUBLIC ${PCL_INCLUDE_DIRS}
)

if(PCL_LIBRARY_DIRS)
    target_link_directories(lvf_core PUBLIC ${PCL_LIBRARY_DIRS})
endif()

target_link_libraries(lvf_core
    PUBLIC
        ${OpenCV_LIBS}
        Eigen3::Eigen
        ${PCL_LIBRARIES}
)

if(PCL_DEFINITIONS)
    target_compile_options(lvf_core PUBLIC ${PCL_DEFINITIONS})
endif()

if(OpenMP_CXX_FOUND)
    target_link_libraries(lvf_core PUBLIC OpenMP::OpenMP_CXX)
endif()

lvf_enable_warnings(lvf_core)

# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------
add_executable(lvf_app app/main.cpp)
set_target_properties(lvf_app PROPERTIES OUTPUT_NAME lidar_fusion)
target_link_libraries(lvf_app PRIVATE lvf_core)
lvf_enable_warnings(lvf_app)

# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------
if(LVF_BUILD_TOOLS)
    add_executable(lvf_deps_check tools/deps_check.cpp)
    target_link_libraries(lvf_deps_check PRIVATE lvf_core)
    lvf_enable_warnings(lvf_deps_check)

    add_executable(lvf_inspect_frame tools/inspect_frame.cpp)
    target_link_libraries(lvf_inspect_frame PRIVATE lvf_core)
    lvf_enable_warnings(lvf_inspect_frame)

    add_executable(lvf_fuse_frame tools/fuse_frame.cpp)
    target_link_libraries(lvf_fuse_frame PRIVATE lvf_core)
    lvf_enable_warnings(lvf_fuse_frame)

    add_executable(lvf_degradation_bench tools/degradation_bench.cpp)
    target_link_libraries(lvf_degradation_bench PRIVATE lvf_core)
    lvf_enable_warnings(lvf_degradation_bench)

    add_executable(lvf_map_frame tools/map_frame.cpp)
    target_link_libraries(lvf_map_frame PRIVATE lvf_core)
    lvf_enable_warnings(lvf_map_frame)
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(LVF_BUILD_TESTS)
    enable_testing()

    function(lvf_add_data_test test_name source_file)
        add_executable(lvf_test_${test_name} ${source_file})
        target_link_libraries(lvf_test_${test_name} PRIVATE lvf_core)
        target_compile_definitions(lvf_test_${test_name} PRIVATE
            LVF_SAMPLE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data/sample")
        lvf_enable_warnings(lvf_test_${test_name})
        add_test(NAME ${test_name} COMMAND lvf_test_${test_name})
    endfunction()

    add_executable(lvf_smoke_test tests/test_smoke.cpp)
    target_link_libraries(lvf_smoke_test PRIVATE lvf_core)
    lvf_enable_warnings(lvf_smoke_test)
    add_test(NAME smoke COMMAND lvf_smoke_test)

    lvf_add_data_test(calibration tests/test_calibration.cpp)
    lvf_add_data_test(kitti_source tests/test_kitti_source.cpp)
    lvf_add_data_test(data_integration tests/test_data_integration.cpp)
    lvf_add_data_test(projection tests/test_projection.cpp)
    lvf_add_data_test(fusion tests/test_fusion.cpp)
    lvf_add_data_test(overlay tests/test_overlay.cpp)
    lvf_add_data_test(pcd_roundtrip tests/test_pcd_roundtrip.cpp)
    lvf_add_data_test(image_degradation tests/test_image_degradation.cpp)
    lvf_add_data_test(lidar_degradation tests/test_lidar_degradation.cpp)
    lvf_add_data_test(degradation_report tests/test_degradation_report.cpp)
    lvf_add_data_test(preprocess tests/test_preprocess.cpp)
    lvf_add_data_test(bev_grid tests/test_bev_grid.cpp)
    lvf_add_data_test(map_io tests/test_map_io.cpp)
    lvf_add_data_test(grid_render tests/test_grid_render.cpp)
    lvf_add_data_test(pipeline tests/test_pipeline.cpp)

    if(TARGET lvf_deps_check)
        add_test(NAME deps_check COMMAND lvf_deps_check)
    endif()

    # On Windows the prebuilt PCL/OpenCV packages keep their DLLs outside the
    # build tree. Prepend those directories only for tests, so `ctest` works
    # without asking the user to edit the system PATH.
    if(WIN32)
        set(LVF_TEST_RUNTIME_DIRS "")

        if(PCL_DIR)
            get_filename_component(_lvf_pcl_root "${PCL_DIR}/.." ABSOLUTE)
            list(APPEND LVF_TEST_RUNTIME_DIRS "${_lvf_pcl_root}/bin")
            list(APPEND LVF_TEST_RUNTIME_DIRS "${_lvf_pcl_root}/3rdParty/FLANN/bin")
            list(APPEND LVF_TEST_RUNTIME_DIRS "${_lvf_pcl_root}/3rdParty/Qhull/bin")
        endif()

        if(VTK_DIR)
            get_filename_component(_lvf_vtk_root "${VTK_DIR}/../../.." ABSOLUTE)
            list(APPEND LVF_TEST_RUNTIME_DIRS "${_lvf_vtk_root}/bin")
        endif()

        if(OpenCV_DIR)
            list(APPEND LVF_TEST_RUNTIME_DIRS "${OpenCV_DIR}/x64/vc16/bin")
        endif()

        set(_lvf_test_env_mods "")
        foreach(_lvf_dir IN LISTS LVF_TEST_RUNTIME_DIRS)
            if(EXISTS "${_lvf_dir}")
                list(APPEND _lvf_test_env_mods "PATH=path_list_prepend:${_lvf_dir}")
            endif()
        endforeach()

        set(_lvf_tests
            smoke
            calibration
            kitti_source
            data_integration
            projection
            fusion
            overlay
            pcd_roundtrip
            image_degradation
            lidar_degradation
            degradation_report
            preprocess
            bev_grid
            map_io
            grid_render
            pipeline
        )
        if(TARGET lvf_deps_check)
            list(APPEND _lvf_tests deps_check)
        endif()

        if(_lvf_test_env_mods)
            set_tests_properties(${_lvf_tests} PROPERTIES
                ENVIRONMENT_MODIFICATION "${_lvf_test_env_mods}")
        endif()
    endif()
endif()
