# Rescue relay package

This repository contains a ROS2 package for a C++ based relay node. It integrates odometry, sensor, audio and video data, and forwards them to the GUI workstation program as compressed packets through a [custom implementation](https://github.com/arepo90/ROTAS) of the RTP protocol. The system implements multithreading with UDP sockets and data compression tools, as it's designed for real-time streaming with minimal latency and required bandwidth.

### Features

- Fully automated processes; once started, manages the GUI connection/disconnection automatically and indefinitely.
- Launcher handles errors and restarts automatically.
- Outsorces computing-intensive tasks to the workstation.
- Creates and handles a virtually limitless number of data channels in parallel.
- Automatic Linux/Windows differentiation.
- In-depth [wiki](wiki.md).

## Dependencies

- ROS2 (Jazzy recommended):
  - `rclcpp`
  - `std_msgs`
  - `sensor_msgs`
  - `geometry_msgs`
  - `nav_msgs`
  - `tf2`
  - `tf2_ros`
  - `tf2_geometry_msgs`
- OpenCV (core)
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
| `/estop`            | `std_msgs/msg/Bool`              | Virtual E-Stop                  |

## Installation & Usage

### Windows

1. Clone repo
```
cd <path_to>/ros2_ws/src
git clone https://github.com/arepo90/rescue_relay.git
```

2. Install dependencies (vcpkg)

> WIP

3. Build package
```
colcon build --merge-install --packages-select rescue_relay
```

4. Source install
```
cd <path_to>/ros2_ws
.\install\setup.bat
```

### Linux

1. Clone repo
```
cd <path_to>/ros2_ws/src
git clone https://github.com/arepo90/rescue_relay.git
```

2. Run setup
```
cd rescue_relay
chmod +x install.sh
./install.sh
```

### Run

```
ros2 launch rescue_relay launch.py
```

Once started, it will await a GUI connection and connect automatically; should it disconnect, it will enter a standby mode while it awaits a new connection. Any crashes or restart calls will close and reinitialize the node automatically.

## Aliases

On the production (Linux) version, the `install.sh` script sets up a series of terminal aliases to facilitate command executions:

| Alias      | Main command                                                | Description                                   |
|------------|-------------------------------------------------------------|-----------------------------------------------|
| `sros`     | `source ./install/setup.bash`                               | Move to ROS2 workspace and source environment |
| `build`    | `colcon build --packages-select rescue_relay`               | Build rescue_relay package                    |
| `relay`    | `ros2 launch rescue_relay launch.py`                        | Start relay launcher                          |
| `runrelay` | `ros2 run rescue_relay relay`                               | Start relay node                              |
| `stop`     | `pkill -9 -f \"relay\"`                                     | Kill relay process remotely                   |
| `topics`   | `ros2 topic list`                                           | List ROS2 topics                              |
| `estop`    | `ros2 topic pub /estop std_msgs/Bool \"data: true\" --once` | Publish virtual E-Stop topic                  |

## Logs

The program regularly outputs messages regarding the state of execution. They are, in increasing order of severity:

- **Info `[i]`**: General information; only appear during the startup and shutdown processes, or when the connection status changes.
- **Warning `[w]`**: Data warnings; generally appear due to unexpected or corrupt data from external sources.
- **Error `[e]`**: Fatal errors that end the program's execution or edge cases due to improper shutdown.

> A blocking function awaiting a response is indicated by `...`

## Notes

- By default, the program trusts the audio and video devices described in the `cam_` and `mic_` vectors, but a scan can be performed to grab current ports.
- This is a _fire-and-forget_ type of program; it's supposed to be started and kept in the background indefinitely.
- The video sources are reserved for the entire duration of the program.
- The first 4 socket ports from `START_PORT` (inclusive) are always used, with increasing pairs proportional to the number of video sources.
- Logs offer a look into the program's execution, but anything other than an `[i] info` message should be treated as an error.
