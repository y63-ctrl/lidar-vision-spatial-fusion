#include "core/Version.h"

#include <iostream>
#include <string>

int main() {
    bool ok = true;
    if (lvf::version().empty()) {
        std::cerr << "[FAIL] version string is empty\n";
        ok = false;
    }
    if (lvf::dependencySummary().find("OpenCV") == std::string::npos) {
        std::cerr << "[FAIL] dependency summary does not mention OpenCV\n";
        ok = false;
    }
    if (lvf::dependencySummary().find("PCL") == std::string::npos) {
        std::cerr << "[FAIL] dependency summary does not mention PCL\n";
        ok = false;
    }
    if (!ok) {
        return 1;
    }

    std::cout << "[PASS] smoke: version=" << lvf::version() << ", deps="
              << lvf::dependencySummary() << ", openmp=" << (lvf::hasOpenMp() ? "on" : "off")
              << '\n';
    return 0;
}
