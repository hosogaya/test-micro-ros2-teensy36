# Using Micro-ROS2 with teensy36

# How to use
## Set up platformio.ini
```ini
# install micro-ros2
lib_deps =
    https://github.com/micro-ROS/micro_ros_platformio
board_microros_distro = galactic # ros2 version
board_microros_transport = serial # transportation
```

## Run Micro-ROS2 Agent and Upload 
Please note that micor-ros2 agent can not detect pre-running node in the micro-computer. So you should launch micro-ros2 agent first, then, upload your project to the micro-computer.

1. Run Micro-ROS2 agent in docker
    ```bash
    # Serial micro-ROS Agent
    docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyACM0 -v6
    ```

2. Upload project
    ```bash
    pio run --target upload
    ```
3. Check connection
    ```bash
        ros2 node list
        # /micro_ros_platformio_node
        ros2 topic list
        # /micro_ros_platformio_node_publisher
    ```

# Install ros2 galactic
https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
```
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
sudo apt install ros-galactic-desktop
```

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