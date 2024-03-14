CONTAINER_NAME := hcrcont
TAG_NAME := imatag
IMAGE_NAME := hcrimg
CURRENT_DIR := $(shell pwd)

stop:
	@docker stop ${CONTAINER_NAME} || true
	@docker rm ${CONTAINER_NAME} || true

build: stop
	@docker build --tag=${IMAGE_NAME}:${TAG_NAME} .

# Once you reach point 4, edit this command to mount your workspace.
run:	
	@xhost +si:localuser:root >> /dev/null
	@docker run \
		-it \
		-e DISPLAY \
		-e ROS_IP=146.169.248.167 \
		--network host \
		-e ROS_MASTER_URI=http://146.169.248.167:11311 \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		--device /dev/snd \
		--privileged \
		-v ${CURRENT_DIR}/ros_ws/:/root/ros_ws/ \
		--name ${CONTAINER_NAME} \
		${IMAGE_NAME}:${TAG_NAME}

exec:
	@docker exec -it ${CONTAINER_NAME} /bin/bash -c\
	"source /opt/ros/noetic/setup.bash && \
	cd /root/ros_ws && catkin_make && \
	source devel/setup.bash && rosdep install 
