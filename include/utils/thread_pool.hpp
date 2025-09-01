// Copyright 2024 coScene
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UTILS__THREAD_POOL_HPP_
#define UTILS__THREAD_POOL_HPP_

#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>
#include <atomic>
#include <iostream>

class ThreadPool {
public:
    explicit ThreadPool(const size_t threads) : stop_(false) {
        create_workers(threads);
    }

    void resize(const size_t new_thread_count) {
        if (new_thread_count == workers_.size()) {
            return;
        }
        
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        if (new_thread_count > workers_.size()) {
            create_workers(new_thread_count - workers_.size());
        } else {
            stop_ = true;
            condition_.notify_all();
            
            for(std::thread &worker: workers_) {
                if(worker.joinable()) {
                    worker.join();
                }
            }
            
            workers_.clear();
            stop_ = false;
            
            create_workers(new_thread_count);
        }
    }

    template<class F>
    auto enqueue(F&& f) -> std::future<decltype(f())> {
        using return_type = decltype(f());
        auto task = std::make_shared<std::packaged_task<return_type()>>(std::forward<F>(f));
        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            if(stop_)
                throw std::runtime_error("enqueue on stopped ThreadPool");
            tasks_.emplace([task](){ (*task)(); });
        }
        condition_.notify_one();  // Use notify_one to reduce wake-up overhead
        return res;
    }

    size_t get_thread_count() const {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        return workers_.size();
    }

    size_t get_queue_size() const {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        return tasks_.size();
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            stop_ = true;
        }
        condition_.notify_all();
        for(std::thread &worker: workers_) {
            if(worker.joinable()) {
                worker.join();
            }
        }
    }

private:
    void create_workers(size_t count) {
        for(size_t i = 0; i < count; ++i) {
            workers_.emplace_back([this] {
                for(;;) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(this->queue_mutex_);
                        this->condition_.wait(lock,
                            [this]{ return this->stop_ || !this->tasks_.empty(); });
                        if(this->stop_ && this->tasks_.empty())
                            return;
                        task = std::move(this->tasks_.front());
                        this->tasks_.pop();
                    }
                    task();
                }
            });
        }
    }

    mutable std::mutex queue_mutex_;
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::condition_variable condition_;
    std::atomic<bool> stop_;
};

#endif  // UTILS__THREAD_POOL_HPP_
