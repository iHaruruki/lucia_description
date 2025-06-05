# lucia_slam_navigation
## Dependency
```shell
$ sudo apt install ros-humble-xacro
$ udo apt install ros-humble-joint-state-publisher
$ sudo apt install ros-humble-joint-state-publisher-gui
```
## Setup
```shell
$ cd ~/ros2_ws/src  #Go to ros workspace
$ git clone https://github.com/iHaruruki/lucia_description.git #clone this package
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source install/setup.bash
```
## Usage
```shell
$ ros2 launch lucia_description robot.launch.py
```
## License
## Authors
## References
