# ros_wit_imu_node
WIT motion's IMU driver node for ROS2. Refer to [wit_imu_driver](https://github.com/strv/wit_imu_driver) in ROS1.

# Installation

To install the ros_wit_imu_node, open a bash terminal, clone the package from Github, and build it:

```bash
cd ~/ros_ws/src/
git clone https://github.com/Ericsii/ros_wit_imu_node.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

# Usage

```bash
cd ~/ros_ws/src/
. ./install/setup.bash # use setup.zsh if using zsh as default shell
ros2 run wit_imu_driver wit_imu_node
```

# Published Topics

- `~/data_raw`: IMU data (`sensor_msg::Imu`)
- `~/temperature`: Temperature from sensor in device (`sensor_msgs::Temperature`)
- `~/mag`: MagneticField (`sensor_msg::MagneticField`)

# Parameters

- `gravity`: Gravity at your site. Default: `9.797673`
- `frame_id`: frame_id of output topics. Default: `imu_link`
- `device`: Device name to communicate. Default : `/dev/ttyUSB0`
- `baud`: Communication baudrate. Default : `9600`

# Services

- `trigger_yaw_clear`: Type `std_srvs::Trigger`
- `trigger_acc_calibration`: Type `std_srvs::Trigger`
- `trigger_mag_calibration`: Type `std_srvs::Trigger`
- `trigger_exit_calibration`: Type `std_srvs::Trigger`

# License

`ros_wit_imu_node` using MIT License

```
MIT License

Copyright (c) 2022 YLFeng

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```