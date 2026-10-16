/**
 * @file ThreadQueue.hpp
 * @brief Thread-safe blocking queue — replaces FreeRTOS queues on Linux.
 *
 * @author Ji Li <liji@bnl.gov>
 * @date 04/04/2026
 * @copyright
 * Copyright (c) 2026 Brookhaven National Laboratory
 * @license BSD 3-Clause License. See LICENSE file for details.
 */
#pragma once

//===========================================================================//

#include <queue>
#include <mutex>
#include <condition_variable>
#include <utility>

//===========================================================================//

template<typename T>
class ThreadQueue {
public:
    /**
     * @brief Push an item and notify one waiting thread.
     */
    void push(T item)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(std::move(item));
        }
        cv_.notify_one();
    }

    /**
     * @brief Pop an item, blocking until one is available.
     */
    T pop()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return !queue_.empty() || stopped_; });
        if (stopped_ && queue_.empty()) {
            return T{};
        }
        T item = std::move(queue_.front());
        queue_.pop();
        return item;
    }

    /**
     * @brief Try to pop an item without blocking.
     * @return true if an item was popped.
     */
    bool try_pop(T& item)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) return false;
        item = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    /**
     * @brief Check if the queue is empty.
     */
    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    /**
     * @brief Signal all waiting threads to wake up and stop.
     */
    void stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopped_ = true;
        }
        cv_.notify_all();
    }

private:
    std::queue<T>               queue_;
    mutable std::mutex          mutex_;
    std::condition_variable     cv_;
    bool                        stopped_ = false;
};

//===========================================================================//
