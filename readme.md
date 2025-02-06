# Motion Planning strategy for multi­agent tethered UAS for exploration of large and confined assets

#### This repo contains the result of my thesis developed in the Department of Electrical and Photonics Engineering at Danish Tecnical University (DTU) as part of the Autonomous Systems Master's program.


## Run simulations

In order to run the simulation localy you need to download and run the coresponding Docker Image. This is an image that contains the entire project including the ROS 2 workspace as well as Isaac Sim and Pegasus Simulator.

```sh

docker pull nikostziaros/thesis_repo:Version1.0

```

Next, run the container

```sh

docker run --name  nikolaostziaros --entrypoint bash -it --gpus all -e  "ACCEPT_EULA=Y" --rm --network=host --privileged  --device /dev/dri  --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"     -e "PRIVACY_CONSENT=Y"     -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw     -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw     -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw     -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw     -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw     -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw     -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw     -v ~/docker/isaac-sim/documents:/root/Documents:rw     -v ~/docker/isaac-sim/documents:/root/Documents:rw     -v /home/isaacsim/s222980/omnigraph:/root/omnigraph:rw     -w /home/ubuntu    nikostziaros/thesis_repo:Version1.0

```

Make sure to give xhost permission to run GUIs inside a docker container.

```
xhost +si:localuser:root

```


Once inside the container source ROS 2 workspace

```

source install/setup.bash

```

