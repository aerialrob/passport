# passport

ROS node for passport

# 1. Installation

## A. Prerequisities

- [ROS](https://www.ros.org/install/)

## B. Installation

```bash
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init

# Add workspace to bashrc.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# Clone repo
cd ~/catkin_ws/src
git clone https://github.com/aerialrob/passport.git

# Install dependencies from rosinstall file using wstool
wstool init
wstool merge passport/install/passport.rosinstall
wstool update
```

Finally, compile:

```bash
# Compile code
catkin build

# Refresh workspace
source ~/.bashrc
```



Check the ESKF resources to Install the required packages in the same workspace

https://github.com/aerialrob/eskf_odometry#readme

# 2. Usage

## roslaunch

The launch includes sensor topics from bag file both ground truth and noisy measurements

```bash
roslaunch passport sensors_sim.launch
```

GPS converter to local odometry

```
roslaunch passport gps_convert.launch
```

Launch Plotjuggler Layout 

```
roslaunch passport plot_launch.launch
```

## Params

Params can be changed from `params/sim_params.yaml`

## ESKF

Check the repo https://github.com/aerialrob/eskf_odometry.git to launch the bag player in the **bag** folder

```
roslaunch eskf_odometry_ros play_bag.launch
```

Launch the eskf node 

```
roslaunch eskf_odometry_ros eskf_odometry_ros.launch
```



