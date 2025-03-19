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
template <typename T>
class atomic_ptr: public std::unique_ptr<T> {

public:
    mutable std::mutex mutex;

    using std::unique_ptr<T>::unique_ptr;
};


/**
 * @brief 优先级线程
 * @param priority - 优先级 (e.g., 0 表长期, 1 表短期)
 */
class PriorityThread: public ThreadPtr {

protected:
    static std::atomic_size_t cnt;
    size_t id, priority;

public:
    PriorityThread() = default;

    template <typename Callable, typename... Args>
    explicit PriorityThread(size_t priority, Callable &&f, Args &&... args
    ): id(++cnt), priority(priority), ThreadPtr(new std::thread(f, args...)) {}

    // 根据优先级和时间戳的排序
    bool operator<(PriorityThread &other) const {
        if (priority != other.priority) {
            // 不同优先级: 高优先级先出
            return priority < other.priority;
        } else {
            // 同优先级: 先入先出
            return id > other.id;
        }
    }

    friend class PriorityThreadPool;
};


// 优先级线程池
class PriorityThreadPool: public std::vector<PriorityThread> {

public:
    PriorityThreadPool() { reserve(thread_pool_size); }

    // 创建线程并入队
    template <typename Callable, typename... Args>
    PriorityThread emplace(size_t priority, Callable &&f, Args &&... args) {
        if (size() >= thread_pool_size) pop(priority);
        // 线程池有空位时
        PriorityThread t(priority, f, args...);
        ScopedLock lock(mutex);
        auto it = std::lower_bound(begin(), end(), t);
        insert(it, t);
        return t;
    }

    // 阻塞等待
    void join(size_t priority = 0) {
        std::vector<PriorityThread> to_remove;
        {
            ScopedLock lock(mutex);
            for (auto it = rbegin(); it != rend(); ++it) {
                if (it->priority < priority) break;
                if ((*it).priority == priority) to_remove.push_back(*it);
            }
        }
        for (auto &t: to_remove) remove(t);
    }

protected:
    std::mutex mutex;

    // 阻塞并删除指定线程
    void remove(PriorityThread &t) {
        if (t->joinable()) t->join();
        ScopedLock lock(mutex);
        auto it = std::lower_bound(begin(), end(), t);
        if (it != end() && *it == t) erase(it);
    }

    // 高优先级线程出队
    void pop(size_t priority) {
        PriorityThread t;
        {
            ScopedLock lock(mutex);
            for (auto it = rbegin(); it != rend(); ++it) {
                // 存在同级别线程 / 已完成线程
                if (it->priority == priority || !(*it)->joinable()) {
                    t = *it;
                    break;
                }
                // 不存在同级别线程
                if (it->priority < priority) {
                    // 多于两倍并发数时, 等待队尾线程
                    if (size() >= 2 * thread_pool_size) t = back();
                    break;
                }
            }
        }
        if (t) remove(t);
    }
};

}

#endif
