
xhost +local:root

docker run --rm -it \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	osrf/ros:foxy-desktop \
	rviz2

xhost -local:root
