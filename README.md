<h2 align="center">PiCAS for ROS2 Galactic</h2>

## 1. Implementation of Priority-Driven Chain-Aware scheduling
### Setup
- This repository includes the PICAS-enabled rclcpp package and some example code that tests PICAS APIs.
- Before building this repo, make sure that your system has ROS 2 packages installed on it (see this [link](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html#) for ROS 2 installation).
- Build repo with PICAS support enabled
  ```
  source /opt/ros/galactic/setup.bash
  colcon build --allow-overriding rclcpp --cmake-args -DPICAS=TRUE
  ```
- Build repo  without PICAS (default rclcpp)
  ```
  source /opt/ros/galactic/setup.bash
  colcon build --cmake-args -DPICAS=FALSE
  ```

### How to use API
- Please see the example code in `ros2-picas-example > picas_example > src > example.cpp` for a single-threaded executor, and `ros2-picas-example > picas_example_mt > src > example_mt.cpp` for a multi-threaded executor
- Create executor and callbacks, e.g., timer_callback, regular_callback, exec1
  ```
  auto timer_callback = std::make_shared<StartNode>("callback_name", "pub_topic_name", trace_callbacks, exe_time(msec), period(msec), false);
  auto regular_callback = std::make_shared<IntermediateNode>("callback_name", "sub_topic_name", "pub_topic_name", trace_callbacks, exe_time(msec), true);
  ```
  Then, 
  ```
  rclcpp::executors::SingleThreadedExecutor exec1; // single-threaded executor
  ```
  or 
  ```
  rclcpp::executors::MultiThreadedExecutor exec1; // multi-threaded executor
  ```
  - `sub_topic_name`: the name of subscription topic
  - `pub_topic_name`: the name of publishing topic
  - `exe_time`: execution time of callback in msec
  - `period`: period of timer callback in msec
  - trace_callbacks: print time stamp as a txt file (see `ros2-picas-example > trace_picas`)
  
- Enable priority-based scheduling
  ```
  exec1.enable_callback_priority();
  ```
- Set executor's rt priority (`SCHED_FIFO` in linux) and CPU assignment
  - For single-threaded executors:
  ```
  exec1.set_executor_priority_cpu(90, 5); // (90: rt priority, 5: cpu #)
  ```  
  - For multi-threaded executors:
  ```
  exec1.cpus = {1, 2, 3}; // CPU1, 2, 3
  exec1.rt_attr.sched_priority = SCHED_FIFO;
  exec1.rt_attr.sched_priority = 80;
  ```
  Other policies such as SCHED_RR and SCHED_DEADLINE are also usable.
- Set priority of callback
  ```
  exec1.set_callback_priority(timer_callback->timer_, 10); // 10: callback's priority (the higher, more critical callback)
  ```
- Finally, spin
  - Single-threaded executor: use `spin_rt` for real-time priority in linux
  ```
  std::thread spinThread1(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec1);
  ```
  - Multi-threaded executor: `spin` assigns each thread's rt priority and cpu affinity based on prespecified params.
  ```
  exec1.spin();
  ```
### Run example
- Use sudo authority when running ros2 package to leverage rt priority in linux system.
  - Modify `/etc/security/limits.conf` as below then reboot the system
```
  <userid>  hard    rtprio  99
  <userid>  soft    rtprio  99
```
- Run example
```
  source install/setup.bash
  ./build/picas_example/example   # single-threaded version

  ./build/picas_example_mt/example_mt   # multi-threaded version
```

**NOTE**: Please reference our ROS2-PiCAS paper that was published in RTAS 2021.
```
@inproceedings{choi2021picas,
  title={PiCAS: New Design of Priority-Driven Chain-Aware Scheduling for ROS2},
  author={Choi, Hyunjong and Xiang, Yecheng and Kim, Hyoseung},
  booktitle={2021 IEEE 27th Real-Time and Embedded Technology and Applications Symposium (RTAS)},
  pages={251--263},
  year={2021},
  organization={IEEE}
}
```
