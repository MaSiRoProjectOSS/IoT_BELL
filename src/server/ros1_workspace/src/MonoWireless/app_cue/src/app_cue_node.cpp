/**
 * @file app_cue_node.cpp
 * @brief MonoWireless Package - TWELITER CUE converter
 * @date 2021-12-30
 *
 * @copyright Copyright (c) 2021-.
 *               MaSiRo Project.
 *
 */
#include "AppCue.h"

#include <ros/ros.h>
#include <std_msgs/String.h>

#define TOPIC_NAME_TWELITE_APP_CUE "TWELITE_app_cue"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, TOPIC_NAME_TWELITE_APP_CUE);
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");
    printf("===============================================================\n");
    ROS_INFO(" start : %s", TOPIC_NAME_TWELITE_APP_CUE);
    printf("===============================================================\n");
    ros::Rate loop_rate(1); // 1Hz
    AppCue    app;

    app.Run(&n);

#if 0
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
#else
    ros::spin();
#endif

    printf("===============================================================\n");
    ROS_INFO(" shutdown : %s", TOPIC_NAME_TWELITE_APP_CUE);
    printf("===============================================================\n");
    return 0;
}
