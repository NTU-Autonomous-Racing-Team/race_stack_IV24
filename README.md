# F1Tenth ICRA 2024
![CleanShot 2024-02-22 at 16 05 55@2x](https://github.com/NTU-Autonomous-Racing-Team/f1tenth_icra2024/assets/65676392/a6afb82f-6c65-4a38-b296-90daf8975e67)

## Installation
Requirements: ROS 2 Foxy, Ubuntu 20.04, Docker.
1. [Install ROS 2 Foxy](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwjV-9Tr4r6EAxVX7TgGHdGDDuwQFnoECBAQAQ&url=https%3A%2F%2Fdocs.ros.org%2Fen%2Ffoxy%2FInstallation.html&usg=AOvVaw3NkQBV1zK8awthVSd0b2X9&opi=89978449)
2. [Setup GitHub crediential](Setup GitHub CLI Manager `https://cli.github.com/manual/`)
3. [Install docker](https://docs.docker.com/engine/install/ubuntu/)
4. [Add Docker into sudo group ](https://docs.docker.com/engine/install/linux-postinstall/)

## Run
### Simulation
```
cd simulator
docker build -f f1tenth_gym_ros.Dockerfile -t f1tenth:gym_ros_foxy .
./run_gym_ros.sh
```

## Car


## System Architecture
