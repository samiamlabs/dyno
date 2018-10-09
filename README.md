# Installing

## Install wstool and rosdep.
```
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build
```


## Create a new workspace in 'catkin_ws'
```
mkdir -p ~/catkin_ws/src
```

## Clone repository
```
cd ~/catkin_ws/src
git clone https://github.com/samiamlabs/dyno.git
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

## Add catkin_ws setup.bash to .bashrc
```
source ~/.bashrc
```

## Using
```
roslaunch dyno_gazebo bringup.launch
```

Open http://io.dynorobotics.se/ in browser

## Optional dependencies
To run a mecanum platform with ros_control:
```
git clone https://github.com/samiamlabs/mecanum_controller
```
To run a tricycle platform with ros_control:
```
git clone https://github.com/samiamlabs/tricycle_controller.git
```

See full usage documentation here: https://dyno-docs.readthedocs.io
