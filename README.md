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

`ros_wit_imu_node` is under the terms of the MIT license.
