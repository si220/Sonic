FROM ros:noetic

# Ignore this for now
SHELL [ "/bin/bash", "-c" ]  

RUN apt-get update && apt-get install -y python3-catkin-tools
RUN apt-get update && apt-get install -y ros-noetic-gazebo-ros
RUN apt-get update && apt-get install -y ros-noetic-pr2-description
RUN apt-get update && apt-get install -y ros-noetic-robot-state-publisher
RUN apt-get update && apt-get install -y ros-noetic-joint-state-publisher
RUN apt-get update && apt-get install -y ros-noetic-navigation
RUN apt-get update && apt-get install -y ros-noetic-rviz
RUN apt-get update && apt-get install -y ros-noetic-teleop-twist-keyboard
RUN apt-get update && apt-get install -y net-tools
RUN apt-get update && apt-get install -y nano
RUN apt-get update && apt-get install -y git
RUN git clone https://github.com/moshulu/aria-legacy.git
RUN cd aria-legacy && make && make install
RUN cd ..
RUN apt-get update && apt-get install -y ros-noetic-map-server ros-noetic-gmapping ros-noetic-slam-gmapping
RUN apt-get update && apt-get install -y ros-noetic-rqt ros-noetic-rqt-graph
RUN apt-get update && apt-get install -y ros-noetic-explore-lite
RUN apt-get update && apt-get install -y ros-noetic-vision-opencv

# CMD provides the default command to execute when starting a container
CMD bash
