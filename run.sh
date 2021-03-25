
xhost +local:root

# osrf/ros:foxy-desktop \
docker run -it \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$(pwd):/home/olepor/dev_ws/src:rw" \
  	pibotros \
	bash

xhost -local:root
