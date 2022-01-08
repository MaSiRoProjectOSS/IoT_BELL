
source /opt/ros/galactic/setup.bash

source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/galactic/
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# ROS Setting
export ROS_DOMAIN_ID=90
export ROS_IP=`hostname -I  | cut -d " " -f 1`
export ROS_MASTER_URI=http://${ROS_IP}:11311
