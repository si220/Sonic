FROM ros:noetic

# Ignore this for now
SHELL [ "/bin/bash", "-c" ]  


RUN apt-get update && apt-get install -y net-tools
RUN apt-get update && apt-get install -y nano
RUN apt-get update && apt-get install -y git
RUN apt-get update && apt-get install -y ros-noetic-rosbridge-suite
RUN apt-get update && apt-get install -y python3-catkin-tools
RUN apt-get update && apt-get install -y python3-usb
RUN apt-get update && apt-get install -y python3-pyaudio
RUN apt-get update && apt-get install -y pulseaudio
RUN apt-get update && apt-get install -y alsa-base
RUN apt-get update && apt-get install -y alsa-utils 
RUN apt-get update && apt-get install -y libsndfile1-dev 
RUN apt-get update && apt-get install -y dkms
RUN apt-get update && apt-get install -y i2c-tools
RUN apt-get update && apt-get install -y libasound2-plugins

# CMD provides the default command to execute when starting a container
CMD bash
