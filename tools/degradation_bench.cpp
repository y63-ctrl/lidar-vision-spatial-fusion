#include "data/KittiDataSource.h"
#include "degradation/DegradationEvaluator.h"
#include "degradation/ImageDegradationCheck.h"
#include "degradation/LidarDegradationCheck.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

double median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    if (values.size() % 2 == 0) {
        return 0.5 * (values[middle - 1] + values[middle]);
    }
    return values[middle];
}

}  // namespace

int main(int argc, char** argv) {
    try {
        std::string dataRoot = "data/sample";
        std::string frameId = "000000";
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--data-root" && i + 1 < argc) {
                dataRoot = argv[++i];
            } else if (arg == "--frame" && i + 1 < argc) {
                frameId = argv[++i];
            } else {
                throw std::invalid_argument("unknown argument: " + arg);
            }
        }

        lvf::KittiDataSource source(dataRoot, {frameId});
        lvf::Frame frame;
        if (!source.next(frame)) {
            throw std::runtime_error("no frame returned");
        }

        lvf::ImageDegradationCheck imageCheck;
        lvf::LidarDegradationCheck lidarCheck;
        lvf::DegradationEvaluator evaluator;

        for (int i = 0; i < 3; ++i) {
            (void)imageCheck.evaluate(frame.image);
            (void)lidarCheck.evaluate(frame);
            (void)evaluator.evaluate(frame);
        }

        std::vector<double> imageTimes;
        std::vector<double> lidarTimes;
        std::vector<double> evaluatorTimes;

        for (int i = 0; i < 20; ++i) {
            auto start = std::chrono::steady_clock::now();
            (void)imageCheck.evaluate(frame.image);
            auto end = std::chrono::steady_clock::now();
            imageTimes.push_back(std::chrono::duration<double, std::milli>(end - start).count());

            start = std::chrono::steady_clock::now();
            (void)lidarCheck.evaluate(frame);
            end = std::chrono::steady_clock::now();
            lidarTimes.push_back(std::chrono::duration<double, std::milli>(end - start).count());

            start = std::chrono::steady_clock::now();
            (void)evaluator.evaluate(frame);
            end = std::chrono::steady_clock::now();
            evaluatorTimes.push_back(
                std::chrono::duration<double, std::milli>(end - start).count());
        }

        std::cout << "frame points: " << frame.cloud->size() << '\n'
                  << "image median: " << median(imageTimes) << " ms\n"
                  << "lidar median: " << median(lidarTimes) << " ms\n"
                  << "evaluator median: " << median(evaluatorTimes) << " ms\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[ERROR] " << error.what() << '\n';
        return 1;
    }
}
