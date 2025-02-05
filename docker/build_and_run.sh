#!/bin/bash

# Set Docker image name
IMAGE_NAME="opencv-ubuntu"
CONTAINER_NAME="sfm_container"
DIR=$(dirname $(pwd))
# Step 1: Build the Docker image
echo "Building Docker image: $IMAGE_NAME..."
docker build -t $IMAGE_NAME .

xhost +
# Step 2: Run the Docker container
echo "Running container: $CONTAINER_NAME..."
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
