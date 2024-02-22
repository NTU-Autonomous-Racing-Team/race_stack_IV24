# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

FROM ros:foxy

SHELL ["/bin/bash", "-c"]

# dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       neovim \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
		       ros-foxy-rviz2

RUN apt-get -y dist-upgrade
RUN pip3 install transforms3d

# f1tenth gym
RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN cd f1tenth_gym && \
    pip3 install -e .

# ros2 gym bridge
RUN mkdir -p sim_ws/src/
RUN 	cd sim_ws/src && \
	git clone https://github.com/f1tenth/f1tenth_gym_ros.git
RUN source /opt/ros/foxy/setup.bash && \
    cd sim_ws &&\
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro foxy -y && \
    colcon build

RUN printf \	
"\n \
source /opt/ros/foxy/setup.bash \n \
source /sim_ws/install/setup.bash \n " \
>> /root/.bashrc
#ros2 launch f1tenth_gym_ros gym_bridge_launch.py" \
#>> /root/.bashrc

#RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
#RUN echo "source /sim_ws/install/setup.bash" >> /root/.bashrc
#RUN echo "ros2 launch f1tenth_gym_ros gym_bridge_launch.py & " >> /root/.bashrc
RUN touch run_sim.sh
RUN chmod +x run_sim.sh
RUN printf \	
"#!/bin/bash \n \
source /opt/ros/foxy/setup.bash \n \
source /sim_ws/install/setup.bash \n \
ros2 launch f1tenth_gym_ros gym_bridge_launch.py" \
>> run_sim.sh

ENTRYPOINT ["/bin/bash"]

#BUILD COMMAND
#docker build -f f1tenth_gym_ros.Dockerfile -t f1tenth:gym_ros_foxy .
