#include "utils/parallel.hpp"

namespace parallel {

unsigned int thread_pool_size = std::thread::hardware_concurrency();
PriorityThreadPool thread_pool;
std::atomic_size_t PriorityThread::mCnt = 0;


void sleep(double seconds) {
  std::this_thread::sleep_for(std::chrono::milliseconds(int(1e3 * seconds)));
}

}
