# Installing

## Install wstool and rosdep.
```
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build
```

## Create a new workspace in 'cartographer_ws'.
```
mkdir cartographer_ws
cd cartographer_ws
wstool init src
```

## Merge the cartographer_ros.rosinstall file and fetch code for dependencies.
```
wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src
```
## Install proto3.
```
src/cartographer/scripts/install_proto3.sh
```

## Install deb dependencies.
The command 'sudo rosdep init' will print an error if you have already
executed it since installing ROS. This error can be ignored.
```
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```

## Build and install cartographer.
```
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash
```

## Add setup.bash to .bashrc
```
source ~/cartographer_ws/devel_isolated/setup.bash >> ~/.bashrc
source ~/.bashrc
```

## Create a new workspace in 'catkin_ws'
```
mkdir -p ~/catkin_ws/src
```

## Clone repository
```
cd ~/catkin_ws/src
git https://github.com/samiamlabs/dyno.git
```

## Install deb dependencies.
```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```

## Build and install.
```
catkin_make
```

## Using
```
roslaunch dyno_gazebo concert_client.launch
```

Open http://io.dynorobotics.se/ in browser
