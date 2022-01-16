#!/bin/bash

startup=true
#startup=false

echo ================================================================
# -------------------------------------------------------------------

if $(echo ${HOSTNAME} | grep -i -n ciro > /dev/null) ; then
    export SKIN_NAME=ciro
elif $(echo ${HOSTNAME} | grep -i -n ciya > /dev/null); then
    export SKIN_NAME=ciya
else
    export SKIN_NAME=masiro
fi
echo "Skin   : ${SKIN_NAME}"


# -------------------------------------------------------------------
ROS_DISTRIBUTIONS=noetic
if [ -e /opt/ros/${ROS_DISTRIBUTIONS} ]; then
    ROS_DISTRIBUTIONS=noetic
else
    # 存在しない場合
    ROS_DISTRIBUTIONS=melodic
fi
source /opt/ros/${ROS_DISTRIBUTIONS}/setup.bash

source /home/akari/IoT_BELL/src/server/ros1_workspace/devel/setup.bash
# -------------------------------------------------------------------
# ###################################################################
export DISPLAY=:0

# -------------------------------------------------------------------
PS_NO=`ps -e -o pid,cmd | grep -E "^.*.launch$" | awk '{print $1}'`
# -------------------------------------------------------------------
echo ================================================================
# -------------------------------------------------------------------

#sleep 20
if [ $startup ]; then
    roslaunch task_manager startup.launch
fi

exit 0
