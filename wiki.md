# Wiki

The following is a full explanation of the `ros2_relay` C++ program.

## Syntax 

In general, different conventions are used to refer to different concepts:

- **ALL_CAPS**: Constant or permanent declarations, generally stated as preprocessor definitions (i.e.: `const` or `#define`).
- **snake_case**: Variables of all kinds, pointers and object instances.
- **camelCase**: Class methods and functions of any kind.
- **PascalCase**: Struct, class, and object names and declarations.

## Terminology

- A **setting** is the initial state of a variable that alters the functioning of a function or the program as a whole, although it's not altered nor modified. 
- A **channel** is an instance of `SocketStruct`, and refers to an object capable of handling two-way data transmission through ROTAS.
- A **payload** is the relevant data or information sent inside a packet.
- A **packet** is an entire data structure meant for transmission. Contains a header and one or more payloads.
- A **header** is a data structure that contains metadata about the rest of the payload, generally placed at the beginning of the packet.
- **Fragmentation** is the process of separating particularly large payloads into sections (fragments) and handling each one as a sub-payload in a different packet.
- A **network port** is a virtual point where a connection starts and/or ends. A socket needs to point to a particular port on any given IP address.
- A **USB port** is a virtual index refering to a physical device connected through USB.
- A **declaration** is the act of creating a variable, object, etc., but not giving it any initial state. 
- An **initialization** is the act of assigning an initial state to a variable, object, etc.
- A **flag** is a boolean or _std::atomic&lt;bool&gt;_ variable that allows a processing loop to be executed (activated = set as _true_, deactivated = set as _false_). The `is_running` flags are activated once on startup and deactivated on shutdown to end all threads, while the `is_active` flags may change throughout the program's execution and serve as a _pause_ or _resume_ indicator.
- **send** refers to all variables, objects, processes, and threads that handle communication from the relay to the GUI.
- **recv** refers to all variables, objects, processes, and threads that handle communication from the GUI to the relay.

## Logs

The program regularly outputs console messages regarding the state of execution. They generally mark the beginning or end of a particularly relevant piece of logic, and are followed by `...` when a blocking function is awaiting a response.

They are, in increasing order of severity:
- **Info `[i]`**: General information; only appear during the startup and shutdown processes, or when the connection status changes.
- **Warning `[w]`**: Data warnings; generally appear due to unexpected or corrupt data from external sources.
- **Error `[e]`**: Fatal errors that end the program's execution or edge cases due to improper shutdown.

> [!TIP]
> The mosts important log messages to watch out for are `[i] Setup done` (program started correctly and is awaiting a connection), `[i] GUI connected` (connection to the GUI has been established and streams should be able to start), `[i] GUI disconnected` (GUI was closed and the program is awaiting a new connetion).

## Overview

`rescue_relay` is a rclcpp package, and as such, it is built using colcon and ran using ROS2. It serves as a middleman or _relay_ between the robot's sensor data on the microcontrollers and the GUI program on the remote workstation.

> [!NOTE]
> With the exception of network sockets, all functions and libraries are cross-platform. The only differences between `ros2_relay.cpp` and `ros2_relay_linux.cpp` are the `#include` headers and the socket implementations inside the _RTPStreamHandler_ class. The OS is automatically detected by CMake during build, so the executable is always called as `relay` regardless of the platform. 

On startup, the program performs the following actions:
1. Start the ROS2 node.
2. Reserve the microphone and video sources.
3. Start the base, audio, and video channels.
4. Subscribe to the ROS2 topics.
5. Start the base, audio and video threads.
6. Await a connection.

It should be noted that audio and video streaming do **not** start immediately after a connection is established; their send threads start with their `is_active` flag deactivated, so their loops wait for a while and skip to the next iteration. It is only after a packet is received and the flag is activated that the actual send logic can be processed.

The GUI is designed to give a notification once when it starts and once again when it shuts down, but the relay itself is not supposed to shut down under any circumstances (unless a fatal error appears). As such, when the GUI disconnects, the relay enters a standby mode while it awaits a new connection.

### Base channel

- send thread:
  1. Copy data from internal buffers.
  2. Assemble packet.
  3. Send packet to GUI
- recv thread:
  1. Await a packet.
  2. Update relevant flags.
  3. Pause/Resume audio and video streams (according to the flags received).
 
### Audio channel

- send thread:
  1. Await `is_active` flag.
  2. Capture audio sample using PortAudio
  3. Encode sample using Opus.
  4. Send to GUI.
- recv thread:
  1. Await a packet.
  2. Update `is_active` flag.
  3. Pause/Resume PortAudio stream (according to the flag received).

> [!NOTE]
> While a deactivated `is_active` flag should skip the execution and prevent audio recording when not needed, PortAudio holds an internal buffer and works in magical and mysterious ways, so manually calling `Pa_StopStream(stream)` to pause and `Pa_StartStream(stream)` to resume is also needed.
 
### Video channels

- send thread:
  1. Await `is_active` flag.
  2. Capture frame from corresponding video source.
  3. Encode frame as _jpeg_ using OpenCV.
  4. Send to GUI.
- recv thread:
  1. Await a packet.
  2. Update `is_active` flag. 

## Initial settings

### ROS2 topics

All preprocessor definitions ending in `_TOPIC` refer to topic subscriptions, with the default ones being:

| Topic name          | Type                             | Description                     |
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

While topic names propagate, more complex settings like data types and callback functions must be modified in the _RelayNode_ class.

### ROTAS

The rest of the preprocessor definitions directly modify the behavior of the ROTAS stream. They **must**:

- `SERVER_IP`: point to the GUI's IP address (should be set up as static).
- `SERVER_PORT`: point to the GUI's start port (and be unused by both devices).
- `FRAGMENTATION_FLAG`: use no more than 4 bytes (works as a reference value; either is or isn't present).
- `MAX_UDP_PACKET_SIZE`: be equal or less than the theoretical maximum size of a UDP datagram, without UDP and RTP headers.
- `AUDIO_SAMPLE_RATE`: be a supported value for Opus and PortAudio (for audio capture and playback; in hertz).
- `AUDIO_FRAME_SIZE`: be a supported value for Opus and PortAudio (for audio fragmentation; in bytes).
- `VIDEO_WIDTH`: be equal or less than the maximum supported horizontal resolution of the video source (for video capture and playback; in pixels).
- `VIDEO_HEIGHT`: be equal or less than the maximum supported vertical resolution of the video source (for video capture and playback; in pixels).

> [!IMPORTANT]
> Excluding the IP address, **all** values must match those found on the GUI.

### Media sources

The `mic_port` and `cam_ports` variables tell the program which physical devices to reserve, and as such must point to valid USB ports. The contents of `cam_names` have no limitations as they only serve as a guide for the GUI, but there **must** be at least as many names as there are `cam_ports`. 

> [!NOTE]
> It should be noted that `cam_ports` serves only as a guide and the actual ports are grabbed from `cam_info`, which is created on `scanPorts()`. 

> [!CAUTION]
> Depending on the setup, the program may not actually check if the ports are valid. While invalid video ports are disregarded and simply give no output, an invalid mic port may throw a segmentation fault on program execution.

## Helpers

- `BasePacket`: Carries all sensor information (that can be stored on the stack) as floats:
  - Robot orientation in degrees (x, y, z angles)
  - Flipper angles in degrees (left, right)
  - Articulation joint angles in degrees (1-4)
  - Track velocities in meters per second (left, right)
  - Magnetometer readings in micro Teslas (x, y, z axis)
  - Gas sensor data in ppm
- `RTPHeader`: Carries all stream metadata needed for transmission. The first five bitfields (`cc`, `x`, `p`, `version`, `pt`) are not actually used but serve as padding. The other values are:
  - `m`: Marker. Contains the total number of fragments for a given payload (same in all fragments).
  - `seq`: Sequence number. Identifies the current fragment for reassembly.
  - `timestamp`: Timestamp. Not actually used in the current implementation.
  - `ssrc`: Synchronization source identifier. Random number unique to each payload (same in all fragments).
- `PayloadType`: Identifies the stream data types as a byte. Not actually used in the current implementation.

The `scanPorts()` function runs once on program startup, and its behavior depends on the flag parameter status:
- **Disabled (default)**: Assumes the data on `cam_ports` is valid and uses it to assemble `cam_info`.
- **Enabled**: Disregards `cam_ports` and instead checks the first five USB ports. Any available device is then added to `cam_info` and anounced through an `[i]` message.

> [!NOTE]
> PortAudio only supports one audio stream at a time. Should an appropiate port not be found after the scan, the port will default to 0.

## RTPStreamHandler

The ROTAS stream. Serves as a universal handler for two-way communication of **any** data types and sizes. 

### Internal variables

- `Stream`: A helper struct that stores the stream information. Not actually used in the current implementation.
- Sockets:
  - `SOCKET`: Send and recv socket objects.
  - `sockaddr_in`: Send and recv socket addresses. Bound to the GUI's ip address and two contiguous ports.
  - `socket_address_size`: Helper variable for `sendto()` and `recvfrom()` socket methods.

### Constructor

An initial port, the target ip address, and the payload type are passed as parameters. 

1. Declares and initializes the `Stream` object (unused).
2. Initializes the `send_socket` object as UDP, and binds `send_socket_address` to the GUI on `port`.
3. Initializes the `recv_socket` object as UDP, starts a temporary 1 MB buffer, and binds the `recv_socket_address` to the GUI on `port+1`.
4. Finishes with an information (`[i]`) message (_Channel created, bound to ports (p, p+1)_).

### Destructor

1. Anounces the call with an `[i]` message.
2. Forcibly shuts down `recv_socket` to prevent a blocking condition.
3. Closes `send_socket` and `recv_socket`.

### destroy()

The actual destructor isn't (and shouldn't) be called as a method. This function serves as an intermediary.

### sendPacket()

Accepts a _std::vector_ of **any** data type as a parameter, which will be encoded as bytes and sent to the GUI.

1. Determines if the payload needs to be fragmented or can be sent in full.
2. Assings a random (not actually but good enough) ssrc to the payload.
3. For every fragment, an `RTPHeader` instance is created and set up. If there is more than one fragment, the `FRAGMENTATION_FLAG` is encoded in the last four bytes of `RTPHeader.seq`.
4. The packet is created as a _std::vector&lt;char&gt;_. The `RTPHeader` is copied first, followed by the current payload fragment.
5. Sends the packet and returns to point 3 if there are multiple packets.

### recvPacket()

Currently only supports non-fragmented packets (large packets are not actually needed from the GUI). 

1. Creates a temporary 4096 bytes buffer.
2. Awaits a packet (blocking function).
3. Payload is parsed as a _std::vector&lt;int&gt;_ and returned (header is disregarded).

> [!NOTE]
> Communication from the GUI to the relay is simple and short, generally consisting of a few flags and indicators; as such, the full implementation of ROTAS is not needed on this side.

## RelayNode

The main executor and ROS2 node. Is spun up on program startup.

### Internal variables

- ROS2 subscriptions: topic subscriptions for all relevant sensor and robot data.
- `_data`: Internal buffers for topic data, as a way to bypass ROS2-ROTAS synchronization.
- `_mutex`: Concurrency control for topic data used to handle get/set operations in different threads.
- `stream`: PortAudio stream pointer initialized on node startup and bound to `mic_port` as input only.
- `opus_encoder`: Opus encoder pointer used to compress audio samples for transmission.
- `_socket`: `SocketStruct` that contains all ROTAS-related variables and objects. Refers to a _channel_.
  - `target_socket`: `RTPStreamHandler` object pointer used for data transmission.
  - `_thread`: Send and recv threads for concurrent two-way communication.
  - `is_`: Thread-safe flags to handle thread execution and shutdown.
 
### Constructor

1. Initializes the base, audio, and stream channels by creating `RTPStreamHandler` instances with the corresponding ports, and setting up the execution flags.
2. Starts the Opus encoder and the PortAudio stream, which starts a recurring audio callback:
   1. If the stream is active, encodes the current audio sample using Opus and sends it through the audio channel.
   2. Otherwise, awaits the active flag.
4. Initializes the base channel send thread, which runs in a loop for the program's lifetime:
   1. Copies ROS2 topic data from the internal buffers using _std::lock_guards_ with their corresponding _std::mutex_. 
   2. Fills empty buffers with 0's to prevent out-of-bounds errors.
   3. Creates a `BasePacket` instance and fills it with the topic data.
   4. Copies the number of video sources to the first four bytes of the packet.
   5. Appends the `BasePacket` data to the packet.
   6. For every video source, appends the name string length and string itself to the payload (for easier reconstruction on GUI).
   7. Appends the thermal sensor data to the payload.
   8. Sends the payload through the base channel.
5. Initializes the base channel recv thread, which runs in a loop for the program's lifetime (more in-depth explanation in the note):
   1.  Awaits a packet from the base channel.
   2.  Expects a `0` or `1` marker, indicating the GUI connected or disconnected, respectively.
   3.  Pauses the audio and video streams.
   4.  Copies the full payload to the internal buffer.
5. Initializes the audio channel recv thread, which runs in a loop for the program's lifetime:
   1. Awaits a packet from the audio channel.
   2. Expects a `0` or `1` marker, indicating a pause or resume to the audio stream, respectively.
6. For every video channel, initializes the send thread, which runs in a loop for the program's lifetime:
   1. Creates a _cv::VideoCapture_ instance bound to the corresponding USB port, and assigns the relevant image settings.
   2. If the stream is active, captures a frame, encodes it using jpeg, and sends it through the corresponding video channel.
   3. Otherwise, awaits the active flag.
7. For every video channel, initializes the recv thread, which runs in a loop for the program's lifetime:
   1. Awaits a paket from the corresponding video channel.
   2. Expects a `0` or `1` marker, indicating a pause or resume to the video stream, respectively.
8. For every ROS2 topic, creates a subscription with a callback function that stores the relevant data in the internal buffers.
9. Finishes with an `[i]` message (_Setup done_).

> [!NOTE]
> The base channel recv thread works on the assumption that all relevant communication from the GUI will be handled by the corresponding channels, and that the base channel is only used once when the GUI starts, and once again when it shuts down. As such, both cases imply a hard reset which is why the audio and video streams are paused regardless of the GUI's state. 

### Destructor

1. Anounces the call with an `[i]` message.
2. Marks the `is_` flags on all channels as _false_, ending the thread executions.
3. Closes the audio stream and frees the Opus object.
4. Joins all channel threads.
5. Finishes with an `[i]` message (_Bye_).

### destroy()

The actual destructor isn't (and shouldn't) be called as a method. This function serves as an intermediary.

### audioCallback()

PortAudio requires a static function callback to work, but the program's implementation requires access to the `RelayNode`'s objects and variables. As such, some magic tricks are done to call `audioProcess()`.

### audioProcess()

Contains the logic needed for the audio stream. Is called recursively after `PaContinue`, so serves as an infinite _while(true)_ loop until the PortAudio stream is closed.

## main()

1. Anounces the call with an `[i]` message (_Hi [Windows/Linux]_).
2. Initializes PortAudio (required for port scans).
3. Runs `scanPorts()`.
4. Initializes ROS2.
5. Initializes the `RelayNode` node.
6. On shutdown, closes ROS2.

> [!TIP]
> The program may sometimes hang after the initial `[i]` message, indicating `rclcpp` is having some trouble starting up. This is generally fixed by waiting for a few seconds or rebuilding the package, so long as no actual error message is shown.
