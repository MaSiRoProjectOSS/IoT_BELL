

# source /opt/ros/foxy/setup.bash
source /opt/ros/noetic/setup.bash

## call "devel setup"
PROJECT_PATH=`echo $HOME`/IoT_BELL
source ${PROJECT_PATH}/src/server/ros1_workspace/devel/setup.bash

# ROS Setting
export ROS_DOMAIN_ID=90
export ROS_IP=`hostname -I  | cut -d " " -f 1`
#export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_MASTER_URI=http://${ROS_IP}:11311
