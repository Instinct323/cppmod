#include "utils/parallel.hpp"

namespace parallel {

unsigned int thread_pool_size = std::max((unsigned int) 4, std::thread::hardware_concurrency());
PriorityThreadPool thread_pool;
std::atomic_size_t PriorityThread::cnt = 0;

}
