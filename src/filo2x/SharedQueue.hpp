#ifndef _FILO2X_SHAREDQUEUE_HPP_
#define _FILO2X_SHAREDQUEUE_HPP_

#include <condition_variable>
#include <list>
#include <mutex>

#include "filo2/base/NonCopyable.hpp"

template <typename T>
class SharedQueue : public cobra::NonCopyable<SharedQueue<T>> {
public:
    SharedQueue() = default;

    T get() {
        std::unique_lock<std::mutex> lock(mutex);
        while (queue.empty()) {
            cond.wait(lock);
        }
        T item = queue.front();
        queue.pop_front();
        return item;
    }

    void push(const T& item) {
        std::unique_lock<std::mutex> lock(mutex);
        queue.emplace_back(item);
        lock.unlock();
        cond.notify_one();
    }

    size_t size() {
        std::unique_lock<std::mutex> lock(mutex);
        return queue.size();
    }

    bool empty() {
        std::unique_lock<std::mutex> lock(mutex);
        return queue.empty();
    }

private:
    std::list<T> queue;
    std::mutex mutex;
    std::condition_variable cond;
};

#endif