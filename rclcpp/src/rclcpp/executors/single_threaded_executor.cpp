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

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/any_executable.hpp"
#include "rclcpp/scope_exit.hpp"

using rclcpp::executors::SingleThreadedExecutor;

SingleThreadedExecutor::SingleThreadedExecutor(const rclcpp::ExecutorOptions &options)
    : rclcpp::Executor(options) {}

SingleThreadedExecutor::~SingleThreadedExecutor() {}

void SingleThreadedExecutor::spin()
{
#ifdef PICAS
  if (cpus.size() <= 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SingleThreadedExecutor: spin (PID %ld): No CPU assigned- CPU 0 as default", gettid());
    cpus = {0};
  }
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(cpus[1], &cpuset);
  if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset))
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "spin in PiCAS has an error (%s)", strerror(errno));
  }

  if (rt_attr.sched_policy != 0)
  {
    long int ret;
    unsigned int flags = 0;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SingleThreadedExecutor: spin (PID %ld) %s prio %d", gettid(),
                rt_attr.sched_policy == SCHED_FIFO ? "FIFO" : rt_attr.sched_policy == SCHED_RR     ? "RR"
                                                          : rt_attr.sched_policy == SCHED_DEADLINE ? "DEADLINE"
                                                                                                   : "N/A",
                rt_attr.sched_priority);
    ret = sched_setattr(0, &rt_attr, flags);
    if (ret < 0)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SingleThreadedExecutor: spin (PID %ld): sched_setattr has an error (%s)", gettid(), strerror(errno));
    }
  }

#endif

  if (spinning.exchange(true))
  {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false););
  while (rclcpp::ok(this->context_) && spinning.load())
  {
    rclcpp::AnyExecutable any_executable;
    if (get_next_executable(any_executable))
    {
      execute_any_executable(any_executable);
    }
  }
}
