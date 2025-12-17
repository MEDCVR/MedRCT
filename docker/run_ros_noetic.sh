sudo docker run -d -it \
 --name ros_noetic \
 -v $HOME:/root \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 -e DISPLAY=$DISPLAY \
 --network host\
 ros:noetic bash
