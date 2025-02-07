# Motion Planning Strategy for Multi-Agent Tethered UAS for Exploration of Large and Confined Assets

This repository contains the results of my thesis developed in the Department of Electrical and Photonics Engineering at the Danish Technical University (DTU) as part of the Autonomous Systems Master's program.

# Video Demonstration


[![Video Demonstration](http://i3.ytimg.com/vi/2MsdKgXXRzo/hqdefault.jpg)](https://youtu.be/2MsdKgXXRzo)


## Running Simulations

To run the simulation locally, you need to download and run the corresponding Docker image. This image contains the entire project, including the ROS 2 workspace, as well as Isaac Sim and the Pegasus Simulator.

### Prerequisites

> **IMPORTANT** We assume that you have Docker installed and access to NVIDIA's Omniverse Launcher, specifically the Streaming Client. Although Isaac Sim will run inside the Docker container, you need the Streaming Client to visualize the simulation. You may need to create an NVIDIA account. You can download the launcher [here](https://developer.nvidia.com/omniverse#section-getting-started).

### Step 1: Pull the Docker Image

Run the following command to download the Docker image:

```sh
docker pull nikostziaros/thesis_repo:Version1.0
```

### Step 2: Run the Docker Container

Execute the command below to start the container:

```sh
docker run --name nikolaostziaros --entrypoint bash -it --gpus all \
  -e "ACCEPT_EULA=Y" --rm --network=host --privileged --device /dev/dri \
  --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -e "PRIVACY_CONSENT=Y" \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/documents:/root/Documents:rw \
  -v /home/isaacsim/s222980/omnigraph:/root/omnigraph:rw \
  -w /home/ubuntu nikostziaros/thesis_repo:Version1.0
```

### Step 3: Enable GUI Permissions

To visualize RVIZ, allow GUI applications to run inside the Docker container:

```sh
xhost +si:localuser:root
```

### Step 4: Launch the Simulation

Once inside the container, navigate to the Pegasus Simulation folder and execute the `demo.py` script to launch Isaac Sim/Pegasus.

```sh
cd ../PegasusSimulator/examples/
ISAACSIM_PYTHON demo.py
```

The script may take a few minutes to complete. After execution, open the Streaming Client from the Omniverse Launcher and press **Connect**. You should now see Isaac Sim running with the environment loaded.

### Step 5: Spawn the ROS 2 Environment

#### 1. Source the ROS 2 workspace:

```sh
source install/setup.bash
```

#### 2. Launch the MoveIt 2 planning scene and visualize it in RVIZ:

```sh
ros2 launch isaac_sim_demo environment.launch.py
```

### Step 6: Start Path Planning

To initiate the path planning process, run:

```sh
ros2 launch isaac_sim_demo environment.launch.py
```


