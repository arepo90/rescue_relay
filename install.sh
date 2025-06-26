#!/bin/bash

echo "[1/4] Installing dependencies..."
sudo apt update
sudo apt install -y \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    portaudio19-dev \
    libopus-dev \
    zlib1g-dev \
    libopencv-dev \
    pkg-config \
    cmake \
    build-essential

echo "[2/4] Building package..."
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-select rescue_relay

echo "[3/4] Setting up aliases..."
grep -q "alias sros=" ~/.bashrc || echo 'alias sros="cd ~/ros2_ws && source ./install/setup.bash"' >> $HOME/.bashrc
grep -q "alias build=" ~/.bashrc || echo 'alias build="cd ~/ros2_ws && colcon build --packages-select rescue_relay"' >> $HOME/.bashrc
grep -q "alias stop=" ~/.bashrc || echo 'alias stop="pkill -9 -f \"relay\""' >> $HOME/.bashrc
grep -q "alias relay=" ~/.bashrc || echo 'alias relay="cd ~/ros2_ws && source ./install/setup.bash && ros2 launch rescue_relay launch.py"' >> $HOME/.bashrc
grep -q "alias runrelay=" ~/.bashrc || echo 'alias runrelay="cd ~/ros2_ws && source ./install/setup.bash && ros2 run rescue_relay relay"' >> $HOME/.bashrc
grep -q "alias topics=" ~/.bashrc || echo 'alias topics="cd ~/ros2_ws && source ./install/setup.bash && ros2 topic list"' >> $HOME/.bashrc
grep -q "alias estop=" ~/.bashrc || echo 'alias estop="ros2 topic pub /estop std_msgs/Bool \"data: true\" --once"' >> $HOME/.bashrc

echo "[4/4] Finalizing..."
source $HOME/.bashrc

echo "Installation complete"
echo "Available commands (aliases):"
echo "  - sros      # Move to ROS2 workspace and source environment"
echo "  - build     # Build rescue_relay package"
echo "  - relay     # Start relay launcher"
echo "  - runrelay  # Start relay node"
echo "  - stop      # Kill relay (any) remotely"
echo "  - topics    # List ROS2 topics"
echo "  - estop     # Virtual E-Stop topic"
