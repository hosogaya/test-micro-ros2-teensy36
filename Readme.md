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
    docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev [YOUR BOARD PORT] -v6
    ```

2. Upload project
    ```bash
    pio run --target upload
    ```
3. Check connection
    ```bash
        ros2 node list
        ros2 topic list
    ```