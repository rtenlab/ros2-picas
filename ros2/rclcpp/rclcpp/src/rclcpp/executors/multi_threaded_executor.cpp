// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/executors/multi_threaded_executor.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

// For ROS2RTF add by HJ
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <string>
#include <sys/syscall.h>

#include "rclcpp/utilities.hpp"
#include "rclcpp/scope_exit.hpp"

using rclcpp::executors::MultiThreadedExecutor;

// Add by HJ
/*
MultiThreadedExecutor::MultiThreadedExecutor(size_t number_of_threads) 
{
  number_of_threads_ = number_of_threads ? number_of_threads : std::thread::hardware_concurrency();
  if (number_of_threads_ == 0) {
    number_of_threads_ = 1;
  }  
}
*/
MultiThreadedExecutor::MultiThreadedExecutor(
  const rclcpp::executor::ExecutorArgs & args,
  size_t number_of_threads,
  bool yield_before_execute,
  std::chrono::nanoseconds next_exec_timeout)
: executor::Executor(args),
  yield_before_execute_(yield_before_execute),
  next_exec_timeout_(next_exec_timeout)
{
  number_of_threads_ = number_of_threads ? number_of_threads : std::thread::hardware_concurrency();
  if (number_of_threads_ == 0) {
    number_of_threads_ = 1;
  }
}

MultiThreadedExecutor::~MultiThreadedExecutor() {}

void
MultiThreadedExecutor::spin()
{
  // Add by HJ
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Number of thread is %d.", this->number_of_threads_);

  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  std::vector<std::thread> threads;
  size_t thread_id = 0;
  {
    std::lock_guard<std::mutex> wait_lock(wait_mutex_);
    for (; thread_id < number_of_threads_ - 1; ++thread_id) {
      auto func = std::bind(&MultiThreadedExecutor::run, this, thread_id);
      threads.emplace_back(func);
    }
  }

  run(thread_id);
  for (auto & thread : threads) {
    thread.join();
  }
}

size_t
MultiThreadedExecutor::get_number_of_threads()
{
  return number_of_threads_;
}

void
MultiThreadedExecutor::run(size_t)
{
  // add by HJ
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TID: %d is MultiThreaded Executor run by ROS2RTF.", syscall(SYS_gettid));

  while (rclcpp::ok(this->context_) && spinning.load()) {
    executor::AnyExecutable any_exec;
    {
      std::lock_guard<std::mutex> wait_lock(wait_mutex_);
      if (!rclcpp::ok(this->context_) || !spinning.load()) {
        return;
      }
      if (!get_next_executable(any_exec, next_exec_timeout_)) {
        continue;
      }
      if (any_exec.timer) {
        // Guard against multiple threads getting the same timer.
        if (scheduled_timers_.count(any_exec.timer) != 0) {
          // Make sure that any_exec's callback group is reset before
          // the lock is released.
          if (any_exec.callback_group) {
            any_exec.callback_group->can_be_taken_from().store(true);
          }
          continue;
        }
        scheduled_timers_.insert(any_exec.timer);
      }
    }
    if (yield_before_execute_) {
      std::this_thread::yield();
    }

    execute_any_executable(any_exec);

    if (any_exec.timer) {
      std::lock_guard<std::mutex> wait_lock(wait_mutex_);
      auto it = scheduled_timers_.find(any_exec.timer);
      if (it != scheduled_timers_.end()) {
        scheduled_timers_.erase(it);
      }
    }
    // Clear the callback_group to prevent the AnyExecutable destructor from
    // resetting the callback group `can_be_taken_from`
    any_exec.callback_group.reset();
  }
}
