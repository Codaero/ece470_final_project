#!/usr/bin/env bash
sudo xhost +local:docker
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]
    then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

DOCKER_USERNAME=ece470

# Specific for NVIDIA drivers, required for OpenGL >= 3.3
docker run -it \
    --rm \
    --name ece484 \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/home/codaero/ece470ws:/home/$DOCKER_USERNAME/ece470ws" \
    -v "/home/codaero/.cache:/home/$DOCKER_USERNAME/.cache" \
    -v "/dev/input:/dev/input" \
    --privileged \
    --security-opt seccomp=unconfined \
    --gpus all \
    --network host \
    codaero/ece470
