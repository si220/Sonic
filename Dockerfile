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
RUN cd aria-legacy && make && make install && cd ..
RUN apt-get update && apt-get install -y ros-noetic-map-server ros-noetic-gmapping ros-noetic-slam-gmapping
RUN apt-get update && apt-get install -y ros-noetic-rqt ros-noetic-rqt-graph
RUN apt-get update && apt-get install -y ros-noetic-explore-lite
RUN apt-get update && apt-get install -y ros-noetic-vision-opencv
RUN sudo apt install python3-pip -y
RUN sudo apt install curl
RUN pip3 install ultralytics opencv-python
RUN pip3 install --force-reinstall numpy==1.24.4
RUN pip3 install --force-reinstall python-dateutil==2.8.2
RUN pip3 install testresources
RUN sudo mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN sudo apt-get install apt-transport-https
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \ 
sudo tee /etc/apt/sources.list.d/librealsense.list
RUN sudo apt-get update
RUN sudo apt-get install librealsense2-dkms -y
RUN sudo apt-get install librealsense2-utils -y
RUN sudo apt-get install librealsense2-dev 
RUN sudo apt-get install librealsense2-dev
RUN sudo apt-get install usbutils -y
RUN pip install pyrealsense2
RUN pip install --force-reinstall lapx==0.5.5
RUN pip install --force-reinstall Cython==3.0.9

# CMD provides the default command to execute when starting a container
CMD bash
