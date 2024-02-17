FROM ros:noetic

# Ignore this for now
SHELL [ "/bin/bash", "-c" ]  

RUN apt-get update && apt-get install -y python3-catkin-tools
RUN apt-get update && apt-get install -y ros-noetic-gazebo-ros
RUN apt-get update && apt-get install -y ros-noetic-pr2-description
RUN apt-get update && apt-get install -y ros-noetic-robot-state-publisher
RUN apt-get update && apt-get install -y ros-noetic-joint-state-publisher
RUN apt-get update && apt-get install -y ros-noetic-rviz
RUN apt-get update && apt-get install -y ros-noetic-ubiquity-motor


# CMD provides the default command to execute when starting a container
CMD bash
