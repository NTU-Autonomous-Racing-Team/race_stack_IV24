# F1Tenth ICRA 2024

![CleanShot 2024-03-14 at 16 42 18@2x](https://github.com/NTU-Autonomous-Racing-Team/f1tenth_icra2024/assets/65676392/44360186-bb67-4fd0-8ce3-2cb304b6a80f)

## Requirements

ROS 2 Foxy, Ubuntu 20.04, Docker.

1. [Install ROS 2 Foxy](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwjV-9Tr4r6EAxVX7TgGHdGDDuwQFnoECBAQAQ&url=https%3A%2F%2Fdocs.ros.org%2Fen%2Ffoxy%2FInstallation.html&usg=AOvVaw3NkQBV1zK8awthVSd0b2X9&opi=89978449)
2. [Setup GitHub crediential](https://cli.github.com/manual/)
3. [Install docker](https://docs.docker.com/engine/install/ubuntu/)
4. [Add Docker into sudo group ](https://docs.docker.com/engine/install/linux-postinstall/)

## Run

### Simulation
```
cd simulator
sudo docker network create f1tenth_network
sudo docker build -f f1tenth_gym_ros.Dockerfile -t f1tenth:gym_ros_foxy .
./run_gym_ros.sh
```
To change map:
1. edit the `f1tenth_gym_ros.Dockerfile` where #change map is.
2. rebuild docker image

### Control Car
To control the car by keyboard, run ```source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker```.

## Documentation

### `simulator/`

The F1tenth simulator. Docker image created from the official simulator. Please don't touch.
## Debug
Sometimes after running the container, the `topics` dont show up with `ros2 topic list`. Try publishing directly to the a topic to see if it is actually there using `ros2 topic pub -r 10 /drive ackermann__msgs/msg/AckermannDriveStamped "{drive: {speed: 0.1}}"`. The car should move

### `f1tenth_ws/`

`f1tenth_system/` f1tenth official pacakge.
` 
`
``

### hardware
1. VESC Setup
  1. Refer to [f1tenth build](https://f1tenth.org/build.html)
  2. Upload Motor Configuration and App Configuration
