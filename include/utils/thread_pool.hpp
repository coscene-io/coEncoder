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
        std::cout << "ThreadPool constructor called with " << threads << " threads" << std::endl;
        // 直接创建指定数量的线程，不调用resize
        create_workers(threads);
        std::cout << "ThreadPool constructor completed, created " << workers_.size() << " threads" << std::endl;
    }

    void resize(const size_t new_thread_count) {
        std::cout << "ThreadPool resize called: " << workers_.size() << " -> " << new_thread_count << std::endl;
        
        if (new_thread_count == workers_.size()) {
            std::cout << "Thread count unchanged, skipping resize" << std::endl;
            return;
        }
        
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        if (new_thread_count > workers_.size()) {
            // 增加线程数量，不需要停止现有线程
            std::cout << "Adding " << (new_thread_count - workers_.size()) << " new threads..." << std::endl;
            create_workers(new_thread_count - workers_.size());
        } else {
            // 减少线程数量，需要停止多余的线程
            std::cout << "Reducing threads from " << workers_.size() << " to " << new_thread_count << std::endl;
            stop_ = true;
            condition_.notify_all();
            
            // 等待所有线程完成当前任务
            for(std::thread &worker: workers_) {
                if(worker.joinable()) {
                    worker.join();
                }
            }
            
            workers_.clear();
            stop_ = false;
            
            // 重新创建指定数量的线程
            create_workers(new_thread_count);
        }
        
        std::cout << "ThreadPool resize completed, now has " << workers_.size() << " threads" << std::endl;
    }

    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) 
        -> std::future<typename std::result_of<F(Args...)>::type> {
        using return_type = typename std::result_of<F(Args...)>::type;

        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
        
        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            if(stop_)
                throw std::runtime_error("enqueue on stopped ThreadPool");
            tasks_.emplace([task](){ (*task)(); });
        }
        condition_.notify_one();
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
        std::cout << "ThreadPool destructor called" << std::endl;
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
        std::cout << "ThreadPool destructor completed" << std::endl;
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
