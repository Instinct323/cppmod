#ifndef UTILS__PARALLEL_HPP
#define UTILS__PARALLEL_HPP

#include <atomic>
#include <algorithm>
#include <memory>
#include <mutex>
#include <thread>


namespace parallel {

class PriorityThreadPool;

typedef std::shared_ptr<std::thread> ThreadPtr;
typedef std::scoped_lock<std::mutex> ScopedLock;

// 硬件并发数
extern unsigned int thread_pool_size;
// 优先级线程池 (优先级 反比于 生命周期)
extern PriorityThreadPool thread_pool;


// 线程共享变量
template<typename T>
class atomic_ptr : public std::unique_ptr<T> {

public:
    mutable std::mutex mutex;

    using std::unique_ptr<T>::unique_ptr;
};


/**
 * @brief 优先级线程
 * @param priority - 优先级 (e.g., 0 表长期, 1 表短期)
 */
class PriorityThread : public ThreadPtr {

protected:
    static std::atomic_size_t cnt;
    size_t timestamp, priority;

public:
    PriorityThread() = default;

    template<typename Callable, typename... Args>
    explicit PriorityThread(size_t priority, Callable &&f, Args &&... args
    ): timestamp(++cnt), priority(priority), ThreadPtr(new std::thread(f, args...)) {}

    // 根据优先级和时间戳的排序
    bool operator<(PriorityThread &other) const {
      if (priority != other.priority) {
        // 高优先级 先出队
        return priority < other.priority;
      } else {
        // 同优先级, 早创建的 先出队
        return timestamp > other.timestamp;
      }
    }

    friend class PriorityThreadPool;
};


// 优先级线程池
class PriorityThreadPool : public std::vector<PriorityThread> {

public:
    PriorityThreadPool() { reserve(thread_pool_size); }

    // 创建线程并入队
    template<typename Callable, typename... Args>
    PriorityThread emplace(size_t priority, Callable &&f, Args &&... args) {
      if (size() >= thread_pool_size) pop(priority);
      // 线程池有空位时
      PriorityThread t(priority, f, args...);
      push_back(t);
      std::sort(begin(), end());
      return t;
    }

    // 阻塞等待
    void join(size_t priority = 0) {
      for (auto it = rbegin(); it != rend(); ++it) {
        if (it->priority < priority) break;
        if ((*it)->joinable()) (*it)->join();
        erase((it + 1).base());
      }
    }

protected:

    // 高优先级线程出队
    void pop(size_t priority) {
      for (auto it = rbegin(); it != rend(); ++it) {
        // 当前优先级存在
        if (it->priority == priority) {
          if ((*it)->joinable()) (*it)->join();
          erase((it + 1).base());
          return;
        }
        // 当前优先级不存在
        if (it->priority < priority) {
          // 多于两倍并发数时, 等待队尾线程
          if (size() >= 2 * thread_pool_size) {
            if (back()->joinable()) back()->join();
            pop_back();
          }
          return;
        }
      }
    }
};

}

#endif
