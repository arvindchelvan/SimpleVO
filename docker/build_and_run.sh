#!/bin/bash

IMAGE_NAME="opencv-ubuntu"
CONTAINER_NAME="vo_container"
DIR=$(dirname $(pwd))

# Build the Docker image
echo "Building Docker image: $IMAGE_NAME..."
docker build -t $IMAGE_NAME .

# Run the Docker container
xhost +

docker run -it --rm \
	   --privileged \
	   --network=host \
	   --env DISPLAY \
           --env QT_X11_NO_MITSHM=1 \
           --env XDG_RUNTIME_DIR=/home/USERNAME/tmp \
           --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
           --volume $HOME/.Xauthority:/root/.Xauthority:rw \
           --pid=host \
	   --name $CONTAINER_NAME \
	   -v "$DIR":/workspace \
	   $IMAGE_NAME
xhost -
