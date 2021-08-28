<h2 align="center">ROS2-PiCAS(foxy)</h2>

### Reason
Because autoware.auto does not have a version based on ros2-eloquent, in order to apply picas to autoware.auto, we need to enable picas to run on foxy.

### Setup
- This repository includes the entire source code of ros2-foxy with PICAS.
- Before building source codes, required development tools and dependencies should be met (please see this [link](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)).

- Source foxy
 ```
  source /opt/ros/foxy/setup.bash
 ```
- Build source codes
  ```
  colcon build --symlink-install
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
