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
                --device=/dev/video2:/dev/video0 \
               --device=/dev/video2:/dev/video1 \
	        --device=/dev/video2:/dev/video2 \
	        --device=/dev/video3:/dev/video3 \
	        --device=/dev/video4:/dev/video4 \
	        --device=/dev/video5:/dev/video5 \
	        --device=/dev/video6:/dev/video6 \
	        --device=/dev/video7:/dev/video7 \
	        --device=/dev/video8:/dev/video8 \
	        --device=/dev/video9:/dev/video9 \
		-it \
		--privileged \
		-e DISPLAY \
<<<<<<< Updated upstream
		-v /dev:/dev \
=======
		-e ROS_IP=146.169.177.188 \
		--network host \
		-e ROS_MASTER_URI=http://146.169.177.188:11311 \
>>>>>>> Stashed changes
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
