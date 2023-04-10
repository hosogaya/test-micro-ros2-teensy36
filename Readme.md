# Using Micro-ROS2 with teensy36

# Requirements
* Ubuntu 20.04
* ROS2 foxy
* micro-ROS
* teensy3.6
* git 

# Install ROS2 foxy
* [Installation guide for ROS2 foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y 
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update && sudo apt upgrade -y
sudo apt install ros-foxy-desktop python3-argcomplete
```

# Install micro-ros2
https://micro.ros.org/docs/tutorials/core/first_application_linux/


## Install Ros2 and the micro-ROS build system
```bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

## Build Ros2 agent
```bash
# Create firmware step
ros2 run micro_ros_setup create_firmware_ws.sh host
# Build step
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh
# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

# Install PlatformIO for VScode
## Install VScode 
* Please download vscode corresponding to your OS from [here](https://code.visualstudio.com/download)

## Install Extensions
* C/C++ Extension Pack (ms-vscode.cpptools-extension-pack)
* ROS (ms-iot.vscode-ros)
* PlatformIO IDE (platformio.platformio-ide)

# Run micro-ROS-agent 
1. Upload this PlatformIO project to teensy36. 
2. Launch micro_ros_agent
```bash
cd <Path-to-your-microros_ws>
source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```
Please change `ttyACM0` to suit your environment. 

## Tests of pub-sub communication with node working on desktop PC
* Baudrate: 921600
* QoS: Best Effort
* MessageType: 'Flaot32MultiArray'
    * Dimension: 2
    * Size: 3 $\times$ 18 or 6 $\times$ 18
1. Build executable for tests.
```bash
cd <Path-to-this-project>/ros2_ws
colcon build
source install/setup.bash
```
2. Launch publisher
```bash
ros2 run test_float32_multi_array test_float32_multi_array_pub
```
Then, data published to `/micro_ros_platformio_node_publisher` are incrimented.

3. Launch subscriber 
```bash
ros2 run test_float32_multi_array test_float32_multi_array_sub
```
Then, the logger will show an datum at the end of the array. 

# Install Docker

## add apt-key and install 
https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-20-04-ja
```
sudo apt update
sudo apt install apt-transport-https ca-certificates curl software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu focal stable"
sudo apt update
sudo apt install docker-ce
```

## check whether succeed to install docker or not
```
sudo systemctl status docker
# expected output 
# docker.service - Docker Application Container Engine
#     Loaded: loaded (/lib/systemd/system/docker.service; enabled; vendor preset: enabled)
#     Active: active (running) since Tue 2020-05-19 17:00:41 UTC; 17s ago
# TriggeredBy: ● docker.socket
#       Docs: https://docs.docker.com
#   Main PID: 24321 (dockerd)
#      Tasks: 8
#     Memory: 46.4M
#     CGroup: /system.slice/docker.service
#             └─24321 /usr/bin/dockerd -H fd:// --containerd=/run/containerd/containerd.sock
```

## execute docker command without sudo 
```
sudo usermod -aG docker ${USER}
su - ${USER}
reboot
```

## Run micro-ROS-agent in docker 
```bash
# Serial micro-ROS Agent
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyACM0 -v6
```