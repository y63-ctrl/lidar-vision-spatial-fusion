#include "utils/Log.h"

#include <iostream>

namespace lvf {

void logInfo(const std::string& message) {
    std::cout << "[INFO] " << message << '\n';
}

void logWarn(const std::string& message) {
    std::cerr << "[WARN] " << message << '\n';
}

void logError(const std::string& message) {
    std::cerr << "[ERROR] " << message << '\n';
}

}  // namespace lvf
