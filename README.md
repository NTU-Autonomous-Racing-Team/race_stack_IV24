# F1Tenth IEEE IV 2024

This repository contains the code that was ran on the car for the F1Tenth IEEE IV 2024 race held in Jeju, Korea. The code here can be directly deployed onto the car.

## Quick Start

### Building the workspace
Clone the repo
```sh
git clone https://github.com/NTU-Autonomous-Racing-Team/race_stack_IV24.git
```
Build the workspace
```sh
cd race_stack_IV24/f1tenth_ws
colcon build
```

### Running Teleop

Connect the controller to the car then run
```sh
ros2 launch f1tenth_stack bringup_launch.py # Default launch file given by F1Tenth
```

To launch the car with safety nodes running
```sh
ros2 launch bringup bringup_safety_NO1.yaml
```

To launch the car with keyboard and joy teleop enabled
```sh
ros2 launch bringup bringup_key_teleop.yaml
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
>NOTE: The commands above should be ran in separate terminals.

- How to run local planners
- How to run global planners

## Software Stack

- Intended system architecture
![CleanShot 2024-03-14 at 16 42 18@2x](https://github.com/NTU-Autonomous-Racing-Team/f1tenth_icra2024/assets/65676392/44360186-bb67-4fd0-8ce3-2cb304b6a80f)
- Architecture we ended up using

