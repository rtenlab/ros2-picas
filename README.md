<h2 align="center">ROS2-PiCAS</h2>

## 1. Implementation of Priority-Driven Chain-Aware scheduling
### Setup
- ROS2-PiCAS is built on [ros2-eloquent](https://docs.ros.org/en/eloquent/Installation/Linux-Development-Setup.html) version.
- This repository includes the entire source code of ros2-eloquent with our priority-based scheduling.
- Before building source codes, required development tools and dependencies should be met (please see this [link](https://docs.ros.org/en/eloquent/Installation/Linux-Development-Setup.html#add-the-ros-2-apt-repository)).
- Build source codes
  ```
  colcon build --symlink-install
  ```

### How to use API
- Please see the example code in `ros2-picas-example > picas_example > src > example.cpp`
- Create executor and callbacks, e.g., timer_callback, regular_callback, exec1
  ```
  auto timer_callback = std::make_shared<StartNode>("callback_name", "pub_topic_name", trace_callbacks, exe_time(msec), period(msec), false);
  auto regular_callback = std::make_shared<IntermediateNode>("callback_name", "sub_topic_name", "pub_topic_name", trace_callbacks, exe_time(msec), true);
  rclcpp::executors::SingleThreadedExecutor exec1;
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
  ```
  exec1.set_executor_priority_cpu(90, 5); // (90: rt priority, 5: cpu #)
  ```  
- Set priority of callback
  ```
  exec1.set_callback_priority(timer_callback->timer_, 10); // 10: callback's priority (the higher, more critical callback)
  ```
- Spin executor (use `spin_rt` for real-time priority in linux)
  ```
  std::thread spinThread1(&rclcpp::executors::SingleThreadedExecutor::spin_rt, &exec1);
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
  ros2 run picas_example example
```

## 2. Analysis framework
### Case study example
- `ros2-picas-example > picas-analysis > picas_casestudy.m`
### Schedulability test
- `ros2-picas-example > picas-analysis > picas_schedulability_test.m`

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