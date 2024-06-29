#ifndef UTILS__PARALLEL_HPP
#define UTILS__PARALLEL_HPP

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
struct SharedVar {
    std::mutex mMutex;
    T mValue;

    SharedVar(T value) : mValue(value) {};

    inline ScopedLock lock() { return ScopedLock(mMutex); }

    operator T() { return mValue; }

    SharedVar<T> &operator=(T value) {
      ScopedLock lock(mMutex);
      mValue = value;
      return *this;
    }
};


/**
 * @brief 优先级线程
 * @param priority - 优先级 (e.g., 0 表长期, 1 表短期)
 */
class PriorityThread : public ThreadPtr {

public:
    static size_t mCnt;

    size_t mTimestamp, mPriority;


    template<typename Callable, typename... Args>
    explicit PriorityThread(size_t priority, Callable &&f, Args &&... args
    ): mTimestamp(++mCnt), mPriority(priority), ThreadPtr(new std::thread(f, args...)) {}

    // 根据优先级和时间戳的排序
    bool operator<(PriorityThread &other) const {
      if (mPriority != other.mPriority) {
        // 高优先级 先出队
        return mPriority < other.mPriority;
      } else {
        // 同优先级, 早创建的 先出队
        return mTimestamp > other.mTimestamp;
      }
    }
};


// 优先级线程池
class PriorityThreadPool : public std::vector<PriorityThread> {

public:
    PriorityThreadPool() { reserve(thread_pool_size); }

    // 阻塞等待
    void join(size_t priority = 0) { while (!empty() && back().mPriority >= priority) pop(); }

    // 高优先级线程出队
    void pop() {
      if (back()->joinable()) back()->join();
      pop_back();
    }

    // 创建线程并入队
    template<typename Callable, typename... Args>
    PriorityThread emplace(size_t priority, Callable &&f, Args &&... args) {
      if (size() >= thread_pool_size) pop();
      // 线程池有空位时
      PriorityThread t(priority, f, args...);
      push_back(t);
      std::sort(begin(), end());
      return t;
    }
};

}

#endif
