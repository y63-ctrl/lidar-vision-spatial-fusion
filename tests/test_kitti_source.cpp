#include "data/KittiDataSource.h"

#include "TestHarness.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

struct TempDirectory {
    std::filesystem::path path;

    ~TempDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }
};

}  // namespace

int main() {
    using namespace lvf::test;

    const std::filesystem::path sampleDir = LVF_SAMPLE_DIR;
    check(std::filesystem::exists(sampleDir), "sample data directory exists");
    check(lvf::KittiDataSource::normalizeFrameId("0") == "000000", "frame id zero-padding");
    check(lvf::KittiDataSource::normalizeFrameId("000000.png") == "000000",
          "frame id extension removal");
    check(lvf::KittiDataSource::normalizeFrameId("000001") == "000001",
          "six-digit frame id is preserved");
    checkThrows([] { lvf::KittiDataSource::normalizeFrameId("a/b"); },
                "frame id with a path separator throws");

    {
        lvf::KittiDataSource source(sampleDir);
        check(source.name() == "KittiDataSource", "source name");

        lvf::Frame frame;
        check(source.next(frame), "first frame is returned");
        check(frame.id == "000000", "discovered frame id");
        check(frame.image.cols == 1242 && frame.image.rows == 375, "image dimensions");
        check(frame.cloud && !frame.cloud->empty(), "cloud is non-empty");
        check(frame.cloud->size() == 20000, "sample cloud point count");
        check(frame.calib.isValid(), "calibration is valid");
        check(!source.next(frame), "source returns false after the last frame");
    }

    {
        const std::filesystem::path missingRoot = sampleDir / "does_not_exist";
        checkThrows([&] {
            lvf::KittiDataSource source(missingRoot, {"000000"});
            lvf::Frame frame;
            source.next(frame);
        }, "missing data root throws");
    }

    {
        TempDirectory temp{std::filesystem::temp_directory_path() / "lvf_kitti_source_bad"};
        std::filesystem::create_directories(temp.path / "image_2");
        std::filesystem::create_directories(temp.path / "velodyne");
        std::filesystem::create_directories(temp.path / "calib");

        std::filesystem::copy_file(sampleDir / "image_2" / "000000.png",
                                   temp.path / "image_2" / "000000.png",
                                   std::filesystem::copy_options::overwrite_existing);
        std::filesystem::copy_file(sampleDir / "calib" / "000000.txt",
                                   temp.path / "calib" / "000000.txt",
                                   std::filesystem::copy_options::overwrite_existing);
        {
            std::ofstream bin(temp.path / "velodyne" / "000000.bin", std::ios::binary);
            const char truncated[15] = {};
            bin.write(truncated, sizeof(truncated));
        }

        checkThrows([&] {
            lvf::KittiDataSource source(temp.path, {"000000"});
            lvf::Frame frame;
            source.next(frame);
        }, "truncated .bin throws");
    }

    {
        lvf::KittiDataSource source(sampleDir, {"0"});
        lvf::Frame frame;
        check(source.next(frame), "normalized frame id can be loaded");
        check(frame.id == "000000", "loaded frame id is normalized");
    }

    {
        std::ifstream input(sampleDir / "calib" / "000000.txt");
        std::stringstream buffer;
        buffer << input.rdbuf();
        std::string text = buffer.str();

        const std::string key = "Tr_velo_to_cam:";
        const auto keyPos = text.find(key);
        check(keyPos != std::string::npos, "sample calib contains Tr_velo_to_cam");
        if (keyPos != std::string::npos) {
            const auto lineEnd = text.find('\n', keyPos);
            text.replace(keyPos, lineEnd - keyPos,
                         "Tr_velo_to_cam: 0 -1 0 0.5 0 0 -1 0 1 0 0 0");
        }

        const auto overridePath =
            std::filesystem::temp_directory_path() / "lvf_kitti_override_calib.txt";
        {
            std::ofstream output(overridePath);
            output << text;
        }

        lvf::KittiDataSource source(sampleDir, {"000000"}, overridePath);
        lvf::Frame frame;
        check(source.next(frame), "calib override frame loads");
        checkNear(frame.calib.T_velo_cam(0, 3), 0.5f, 1e-6f,
                  "calib override is used instead of the per-frame calib");

        std::error_code ignored;
        std::filesystem::remove(overridePath, ignored);
    }

    return finish("kitti_source");
}
