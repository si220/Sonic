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
		--privileged \
		--network host \
		-v /dev:/dev \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v ${CURRENT_DIR}/ros_ws/:/root/ros_ws/ \
		--name ${CONTAINER_NAME} \
		${IMAGE_NAME}:${TAG_NAME} \
		/bin/bash -c "source /opt/ros/noetic/setup.bash \
		&& cd /root/ros_ws && catkin_make \
		&& source devel/setup.bash && rosdep install \
		--from-paths src --ignore-src --rosdistro \
		noetic -y && bash"

exec:
	@docker exec -it ${CONTAINER_NAME} /bin/bash -c \
	"source /opt/ros/noetic/setup.bash && \
	cd /root/ros_ws && catkin_make && \
	source devel/setup.bash && rosdep install \
	--from-paths src --ignore-src --rosdistro \
	noetic -y && bash"