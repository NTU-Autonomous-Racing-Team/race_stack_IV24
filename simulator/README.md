# f1tenth gym ros docker

## Requirements

- Docker
- Linux

## Getting started

1. Download the repository `git clone https://github.com/NTU-Autonomous-Racing-Team/f1tenth_icra2024.git`
2. Setup GitHub CLI Manager `https://cli.github.com/manual/`
3. [Install docker](https://docs.docker.com/engine/install/ubuntu/)
4. [Add Docker into sudo group ](https://docs.docker.com/engine/install/linux-postinstall/)
5. Build the image `docker build -f f1tenth_gym_ros.Dockerfile -t f1tenth:gym_ros_foxy .`
6. Start the container `./run_gym_ros.sh` and RVIZ will pop up.
7. If you need a terminal session of the container, run `./run_gym_ros.sh` again.
8. Containers are deleted on exit.
