#include <thread>

#include "utils/std.hpp"

namespace std {

void sleep(double seconds) {
  std::this_thread::sleep_for(std::chrono::milliseconds(int(1e3 * seconds)));
}

}
