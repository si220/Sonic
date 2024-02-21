FROM ros:kinetic

# Ignore this for now
SHELL [ "/bin/bash", "-c" ]  

RUN apt-get update && apt-get install -y ros-kinetic-catkin
RUN apt-get update && apt-get install -y ros-kinetic-gazebo-ros
RUN apt-get update && apt-get install -y ros-kinetic-pr2-description
RUN apt-get update && apt-get install -y ros-kinetic-robot-state-publisher
RUN apt-get update && apt-get install -y ros-kinetic-joint-state-publisher
RUN apt-get update && apt-get install -y ros-kinetic-rviz
RUN apt-get update && apt-get install -y ros-kinetic-ubiquity-motor
RUN apt-get update && apt-get install -y ros-kinetic-magni-robot
RUN apt-get update && apt-get install -y ros-kinetic-teleop-twist-keyboard
RUN apt-get update && apt-get install -y libnss-mdns avahi-daemon avahi-utils
RUN apt-get update && apt-get install -y iputils-ping
RUN apt-get update && apt-get install -y openssh-client
RUN apt-get update && apt-get install -y chrony
RUN apt-get update && apt-get install -y gedit
RUN apt-get update && apt-get upgrade -y

# CMD provides the default command to execute when starting a container
CMD bash
