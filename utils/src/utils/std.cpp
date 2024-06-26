#include "utils/std.hpp"

namespace std {

void sleep(double seconds) {
  boost::this_thread::sleep_for(boost::chrono::milliseconds(int(1e3 * seconds)));
}

}
