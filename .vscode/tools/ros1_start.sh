#!/bin/bash

## ================================
## Escape sequence
COLOR_ON_RED="\e[31m"
COLOR_OFF="\e[m"
##
## ================================

WORK_FOLDER=$1

if [ "ros_noetic" != "ros_${ROS_DISTRO}" ];
then
    if [ "ros_melodic" != "ros_${ROS_DISTRO}" ];
    then
        echo -e "${COLOR_ON_RED}========================================${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}  Not support Distributions : ${ROS_DISTRO}.${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}========================================${COLOR_OFF}"
        exit 1
    fi
fi

if [ ! -z "${WORK_FOLDER}" ]
then
    if [ -d "${WORK_FOLDER}" ]
    then
        cd ${WORK_FOLDER}

        source /opt/ros/noetic/setup.bash
        echo -e "========================================"
        echo -e "Folder : ${WORK_FOLDER}"
        echo -e "========================================"
        source ${WORK_FOLDER}/setup.bash
        roscore
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
