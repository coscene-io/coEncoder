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

#ifndef SINGLETON_LOCK_HPP
#define SINGLETON_LOCK_HPP

#include <string>
#include <functional>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>

namespace coscene {

/**
 * @brief Singleton lock class to ensure only one instance of a process runs
 * 
 * This class provides a file-based locking mechanism to ensure that only
 * one instance of a process can run at a time. It uses flock() system call
 * to create an exclusive lock on a file.
 */
class SingletonLock {
public:
    /**
     * @brief Constructor
     * @param lock_name Unique name for the lock file (will be prefixed with /tmp/)
     * @param on_exit_callback Optional callback function to be called when lock is released
     */
    inline explicit SingletonLock(const std::string& lock_name, std::function<void()> on_exit_callback = nullptr)
        : lock_file_path_("/tmp/" + lock_name + ".lock")
        , lock_fd_(-1)
        , is_locked_(false)
        , on_exit_callback_(on_exit_callback)
        , on_signal_callback_(nullptr) {}
    
    /**
     * @brief Destructor - automatically releases the lock
     */
    inline ~SingletonLock() { release(); }
    
    /**
     * @brief Try to acquire the singleton lock
     * @return true if lock was acquired successfully, false otherwise
     */
    inline bool acquire() {
        if (is_locked_) {
            return true; // Already locked
        }
        lock_fd_ = open(lock_file_path_.c_str(), O_CREAT | O_RDWR, 0644);
        if (lock_fd_ == -1) {
            std::cerr << "Failed to open lock file: " << strerror(errno) << std::endl;
            return false;
        }
        if (flock(lock_fd_, LOCK_EX | LOCK_NB) == -1) {
            if (errno == EWOULDBLOCK) {
                std::cerr << "Another instance is already running" << std::endl;
            } else {
                std::cerr << "Failed to acquire lock: " << strerror(errno) << std::endl;
            }
            close(lock_fd_);
            lock_fd_ = -1;
            return false;
        }
        char pid_str[32];
        snprintf(pid_str, sizeof(pid_str), "%d\n", getpid());
        if (write(lock_fd_, pid_str, strlen(pid_str)) == -1) {
            std::cerr << "Failed to write PID to lock file: " << strerror(errno) << std::endl;
        }
        is_locked_ = true;
        std::cout << "Singleton lock acquired successfully" << std::endl;
        return true;
    }
    
    /**
     * @brief Release the singleton lock
     */
    inline void release() {
        if (is_locked_ && lock_fd_ != -1) {
            flock(lock_fd_, LOCK_UN);
            close(lock_fd_);
            lock_fd_ = -1;
            is_locked_ = false;
            std::cout << "Singleton lock released" << std::endl;
            if (on_exit_callback_) {
                on_exit_callback_();
            }
        }
    }
    
    /**
     * @brief Check if the lock is currently held
     * @return true if lock is held, false otherwise
     */
    inline bool is_locked() const { return is_locked_; }
    
    /**
     * @brief Get the process ID of the current lock holder
     * @return Process ID, or -1 if no lock is held
     */
    inline int get_lock_pid() const {
        if (!is_locked_) {
            return -1;
        }
        std::ifstream file(lock_file_path_);
        if (!file.is_open()) {
            return -1;
        }
        std::string pid_str;
        if (std::getline(file, pid_str)) {
            try {
                return std::stoi(pid_str);
            } catch (const std::exception&) {
                return -1;
            }
        }
        return -1;
    }
    
    /**
     * @brief Set up signal handlers for graceful shutdown
     * @param on_signal_callback Optional callback to be called on signal
     */
    inline void setup_signal_handlers(std::function<void()> on_signal_callback = nullptr) {
        on_signal_callback_ = on_signal_callback;
        g_singleton_instance = this;
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGABRT, signal_handler);
    }

private:
    std::string lock_file_path_;
    int lock_fd_;
    bool is_locked_;
    std::function<void()> on_exit_callback_;
    std::function<void()> on_signal_callback_;
    
    // Disable copy constructor and assignment operator
    SingletonLock(const SingletonLock&) = delete;
    SingletonLock& operator=(const SingletonLock&) = delete;

    // For signal handler
    static SingletonLock* g_singleton_instance;
    static void signal_handler(int signal) {
        if (g_singleton_instance) {
            if (g_singleton_instance->on_signal_callback_) {
                g_singleton_instance->on_signal_callback_();
            }
            g_singleton_instance->release();
        }
        exit(signal);
    }
};

// Definition of static member
inline SingletonLock* SingletonLock::g_singleton_instance = nullptr;

} // namespace coscene

#endif // SINGLETON_LOCK_HPP 