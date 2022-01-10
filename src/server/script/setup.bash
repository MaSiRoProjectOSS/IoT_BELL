
source /opt/ros/galactic/setup.bash

source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/galactic/
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# ROS Common Setting
export ROS_HOME=/var/tmp/ros
export ROS_LOG_DIR=/var/tmp/ros/logs
#export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1  # 1 for forcing it
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1

# ROS Setting
export ROS_DOMAIN_ID=90
export ROS_IP=`hostname -I  | cut -d " " -f 1`
export ROS_MASTER_URI=http://${ROS_IP}:11311
