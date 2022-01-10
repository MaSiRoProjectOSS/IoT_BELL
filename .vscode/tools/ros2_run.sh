#!/bin/bash

## ================================
## Escape sequence
COLOR_ON_RED="\e[31m"
COLOR_OFF="\e[m"
##
## ================================

WORK_FOLDER=$1
PACKAGES_SELECT=$2
NODE_NAME=$3

if [ "ros_galactic" != "ros_${ROS_DISTRO}" ];
then
    if [ "ros_foxy" != "ros_${ROS_DISTRO}" ];
    then
        if [ "ros_rolling" != "ros_${ROS_DISTRO}" ];
        then
            echo -e "${COLOR_ON_RED}========================================${COLOR_OFF}"
            echo -e "${COLOR_ON_RED}  Not support Distributions : ${ROS_DISTRO}.${COLOR_OFF}"
            echo -e "${COLOR_ON_RED}========================================${COLOR_OFF}"
            exit 1
        fi
    fi
fi

if [ ! -z "${WORK_FOLDER}" ]
then
    if [ -d "${WORK_FOLDER}" ]
    then
        cd ${WORK_FOLDER}

        source /usr/share/colcon_cd/function/colcon_cd.sh
        export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/
        source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
        source /opt/ros/${ROS_DISTRO}/setup.bash
        source ${WORK_FOLDER}/install/setup.bash

        echo -e "========================================"
        echo -e "  ros2 run ${PACKAGES_SELECT} ${NODE_NAME}"
        echo -e "========================================"
        ros2 run ${PACKAGES_SELECT} ${NODE_NAME}
    else
        echo -e "${COLOR_ON_RED}========================================${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}  Could not find the folder : ${WORK_FOLDER}${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}========================================${COLOR_OFF}"
    fi
else
    echo -e "${COLOR_ON_RED}========================================${COLOR_OFF}"
    echo -e "${COLOR_ON_RED}  Specify a folder as the argument.${COLOR_OFF}"
    echo -e "${COLOR_ON_RED}========================================${COLOR_OFF}"
fi
