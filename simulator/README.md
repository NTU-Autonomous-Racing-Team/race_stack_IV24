# F1tenth Simulator 
## Setup
1. Build Dockerfile
```
cd simulator
sudo docker build -f f1tenth_gym_ros.Dockerfile -t f1tenth:gym_ros_foxy .
```
2. Install dependencies
```
sudo apt install python3-pip
git clone https://github.com/f1tenth/f1tenth_gym
cd f1tenth_gym && pip3 install -e .
```
4. Start container to visualise simulator
```
./run_gym_ros.sh
```
4. Start container to access `f1tenth_ws`
```
./run_gym_ros.sh # starts a new session in container
```
5. Go to f1tenth_ws in container
```
cd ~/f1tenth_ws
```

## Troubleshoot
1. Test if topic are publishing
   1. Test: `ros2 topic list`
   2. Expected:
      ```
      root@f1tenth_gym_ros:/# ros2 topic list
      /clicked_point
      /clock
      /cmd_vel
      /drive
      /ego_racecar/odom
      /ego_robot_description
      /goal_pose
      /initialpose
      /joint_states
      /map
      /map_server/transition_event
      /map_updates
      /parameter_events
      /rosout
      /scan
      /tf
      /tf_static
      ```
   

## To change map:
1. edit the `f1tenth_gym_ros.Dockerfile` where #change map is.
2. rebuild docker image

## Resources
- [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros.git)
- [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks.git)
