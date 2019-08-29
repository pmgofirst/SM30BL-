#!/bin/bash
### BEGIN INIT INFO
# Provides: steeringstart
# Short-Description:mlkk steeringstart
# Description:
# Required-Start: $remote_fs $local_fs
# Required-Stop: $remote_fs $local_fs
# Default-Start: 2 3 4 5
# Default-Stop: 0 1 6
### END INIT INFO
source /opt/ros/kinetic/setup.bash
source /home/cc/catkin_ws/devel/setup.bash
roslaunch px4flow steeringcontrol.launch
exit 0
