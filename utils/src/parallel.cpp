#include "utils/parallel.hpp"

namespace parallel {

unsigned int thread_pool_size = std::thread::hardware_concurrency();
PriorityThreadPool thread_pool;
std::atomic_size_t PriorityThread::mCnt = 0;

}
