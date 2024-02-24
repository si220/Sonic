# Sonic - The robot hearing dog

## ELEC70015 Human Centred Robotics module at Imperial College London

## Usage

### Install Docker- instructions can be found on the [Docker](https://www.docker.com/) website

### Clone this repo with the following command which will also clone the required submodules:

```git clone --recurse-submodules https://github.com/si220/Sonic.git```

### To build the docker container run the following command:

```sudo make build```

### To run the container run the following command:

```sudo make run```

### To launch the gazebo simulation environment run:

```roslaunch map_simulation map_simulation.launch```

### To run the slam algorithm:

```rosrun gmapping slam_gmapping scan:=/hokuyo_scan```

### To make the robot explore:

```roslaunch explore_lite explore.launch```

### To run other ros commands from another terminal run:

```sudo make exec```

### To exit the container either press `ctrl+d` or type `exit`

### To stop the container run the following command:

```sudo make stop```
