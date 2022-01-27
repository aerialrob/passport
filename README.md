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

# 2. Usage

## rosrun

Cpp:
```bash
rosrun passport passport
```

Python:
```bash
rosrun passport passport_script
```

## roslaunch
```bash
roslaunch passport passport
```

## rosservice
