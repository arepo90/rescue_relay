# Rescue relay package

This repository contains a ROS2 package for a C++ based relay node. It integrates odometry, sensor, audio and video data, and forwards them to the GUI workstation program as compressed packets through a [custom implementation](https://github.com/arepo90/ROTAS) of the RTP protocol. The system implements multithreading with UDP sockets and data compression tools, as it's designed for real-time streaming with minimal latency and required bandwidth.

### Features

- Fully automated processes; once started, manages the GUI connection/disconnection automatically and indefinitely.
- Outsorces computing-intensive tasks to the workstation.
- Creates and handles a virtually limitless number of data channels in parallel.
- Automatic Linux/Windows differentiation on CMakeLists.
- In-depth [wiki](wiki.md).

## Dependencies

- ROS2 (Jazzy recommended):
  - `rclcpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`
- Windows (Vcpkg):
  - OpenCV 
  - PortAudio
  - Opus
  - FFmpeg
  - ZLib
- Linux:
  - OpenCV core (4.10.x or newer)
  - PortAudio 
  - Opus

## ROS2 subscriptions

| Topic               | Type                             | Description                     |
|---------------------|----------------------------------|---------------------------------|
| `/imu_data`         | `sensor_msgs/msg/Imu`            | BNO055 sensor data              |
| `/track_velocity`   | `geometry_msgs/msg/Vector3`      | Track velocities (z is unused)  |
| `/magnetometer`     | `geometry_msgs/msg/Vector3`      | Magnetometer data               |
| `/thermal_image`    | `std_msgs/msg/Float32MultiArray` | 8x8 thermal camera image        |
| `/encoder_position` | `std_msgs/msg/Float32`           | Flipper angles                  |
| `/mq2_gas`          | `std_msgs/msg/Float32`           | Gas sensor data                 |
| `/joint_base`       | `std_msgs/msg/Float32`           | Arm base joint angle            |
| `/joint_shoulder`   | `std_msgs/msg/Float32`           | Arm shoulder joint angle        |
| `/joint_elbow`      | `std_msgs/msg/Float32`           | Arm elbow joint angle           |
| `/joint_hand`       | `std_msgs/msg/Float32`           | Arm gripper angle               |

## Installation & Usage

### Windows

1. Clone repo
```
cd <path_to>/ros2_ws/src
git clone https://github.com/arepo90/rescue_relay.git
cd rescue_relay
```

2. Build package
```
colcon build --merge-install --packages-select rescue_relay
```

3. Source install
```
.\install\setup.bat
```

### Linux

1. Clone repo
```
cd <path_to>/ros2_ws/src
git clone https://github.com/arepo90/rescue_relay.git
cd rescue_relay
```

2. Build package
```
colcon build --packages-select rescue_relay
```

3. Source install
```
source install/setup.bash
```

### Run

```
ros2 run rescue_relay relay
```

Once started, it will await a GUI connection and connect automatically. Should it disconnect, it will enter a standby mode while it awaits a new connection.

## Logs

The program regularly outputs messages regarding the state of execution. They are, in increasing order of severity:

- **Info `[i]`**: General information; only appear during the startup and shutdown processes, or when the connection status changes.
- **Warning `[w]`**: Data warnings; generally appear due to unexpected or corrupt data from external sources.
- **Error `[e]`**: Fatal errors that end the program's execution or edge cases due to improper shutdown.

> A blocking function awaiting a response is indicated by `...`

## Notes

- By default, it scans and selects the available cameras, as well as the default audio device. You can modify the ports manually through the `cam_` and `mic_` vectors.
- This is a _fire-and-forget_ type of program; it's supposed to be started and kept in the background indefinitely.
- The video sources are reserved for the entire duration of the program.
- The first 4 socket ports from `START_PORT` (inclusive) are always used, with increasing pairs proportional to the number of video sources.
- Logs offer a look into the program's execution, but anything other than an `[i] info` message should be treated as an error.

---
