#include <algorithm>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>

template <typename T> class Queue {
  public:
    Queue() = default;
    // delete copying
    Queue(const Queue<T>&) = delete;

    virtual ~Queue() {}

    unsigned long size() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return queue_.size();
    }

    T pop() {
        std::unique_lock<std::mutex> lk(mutex_);
        if (queue_.empty()) {
            con_v.wait(lk, [this] { return count_ > 0; });
        }
        T tmp = queue_.front();
        queue_.pop();
        count_--;
        lk.unlock();
        return tmp;
    }

    void push(const T& item) {
        {
            std::lock_guard<std::mutex> lk(mutex_);
            queue_.push(item);
            count_++;
        }
        con_v.notify_one();
    }

  private:
    std::queue<T> queue_;
    int count_;
    mutable std::mutex mutex_;
    std::condition_variable con_v;
};
