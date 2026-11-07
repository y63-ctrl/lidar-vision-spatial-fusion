#pragma once

#include <cmath>
#include <iostream>
#include <string>
#include <utility>

namespace lvf::test {

inline int failures = 0;

inline void check(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "[FAIL] " << message << '\n';
        ++failures;
    }
}

inline void checkNear(float actual, float expected, float tolerance, const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << "[FAIL] " << message << " (actual=" << actual << ", expected=" << expected
                  << ", tolerance=" << tolerance << ")\n";
        ++failures;
    }
}

template <typename Callable>
void checkThrows(Callable&& callable, const std::string& message) {
    try {
        std::forward<Callable>(callable)();
    } catch (const std::exception&) {
        return;
    } catch (...) {
        return;
    }
    std::cerr << "[FAIL] " << message << " (no exception thrown)\n";
    ++failures;
}

inline int finish(const char* testName) {
    if (failures == 0) {
        std::cout << "[PASS] " << testName << '\n';
        return 0;
    }
    std::cerr << "[FAIL] " << testName << " (" << failures << " failure(s))\n";
    return 1;
}

}  // namespace lvf::test
