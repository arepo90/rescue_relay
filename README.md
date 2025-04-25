# Rescue relay package

This repository contains a ROS2 package for a C++ based relay node. It integrates odometry, sensor, audio and video data, and forwards them to the GUI workstation program as compressed packets through a [custom implementation](https://github.com/arepo90/ROTAS) of the RTP protocol. The system implements multithreading with UDP sockets and data compression tools, as it's designed for real-time streaming with minimal latency and required bandwidth.

### Features

- Fully automated processes; once started, manages the GUI connection/disconnection automatically and indefinitely.
- Outsorces computing-intensive tasks to the workstation.
- Creates and handles a virtually limitless number of data channels in parallel.

## Dependencies

- ROS2 (Humble or newer):
  - `rclcpp`, `sensor_msgs`, `nav_msgs`, `sensor_msgs`, `tf2_ros`
- Vcpkg (Windows):
  - OpenCV (4.10.x or newer)
  - PortAudio (19.7)
  - Opus (1.5.2)

## ROS2 subscriptions

| Topic            | Type                          | Description                             |
|------------------|-------------------------------|-----------------------------------------|
| `/motors_info`   | `std_msgs/msg/String`         | Parsed string with velocities and RPMs  |
| `/odom`          | `nav_msgs/msg/Odometry`       | Computed robot odometry                 |
| `/joint_states`  | `sensor_msgs/msg/JointState`  | Articulation positions                  |
| `/sensor?`       | `?`                           | ?                                       |

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
source install/setup.bashTh
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

- This is a _fire-and-forget_ type of program; it's supposed to be started and kept in the background indefinitely.
- The video sources are reserved for the entire duration of the program.
- The first 4 socket ports from `START_PORT` (inclusive) are always used, with increasing pairs proportional to the number of video sources.
- Logs offer a look into the program's execution, but anything other than an `[i] info` message should be treated as an error.

---
