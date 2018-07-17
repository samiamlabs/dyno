# Installation

cd ~/catkin_ws/src
git https://github.com/samiamlabs/dyno.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y -r
catkin_make
