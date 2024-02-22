# F1Tenth ICRA 2024
![CleanShot 2024-02-22 at 16 05 55@2x](https://github.com/NTU-Autonomous-Racing-Team/f1tenth_icra2024/assets/65676392/a6afb82f-6c65-4a38-b296-90daf8975e67)

## Installation
Requirements: ROS 2 Foxy, Ubuntu 20.04.


## Setup
## Script: Simulation
- [Install docker](https://docs.docker.com/engine/install/ubuntu/)
- [Add Docker into sudo group ](https://docs.docker.com/engine/install/linux-postinstall/)

```
cd simulator
docker build -f f1tenth_gym_ros.Dockerfile -t f1tenth:gym_ros_foxy .
./run_gym_ros.sh
```

## Script: Car


## System Architecture
