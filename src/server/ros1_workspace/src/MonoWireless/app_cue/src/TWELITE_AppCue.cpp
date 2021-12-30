/**
 * @file TWELITE_AppCue.cpp
 * @brief
 * @date 2021-12-30
 *
 * @copyright Copyright (c) 2021-.
 *               MaSiRo Project.
 *
 */

#include "TWELITE_AppCue.h"

/**
 * @brief Construct a new twelite app cue::twelite app cue object
 *
 */
TWELITE_app_cue::TWELITE_app_cue() {}
/**
 * @brief Destroy the twelite app cue::twelite app cue object
 *
 */
TWELITE_app_cue::~TWELITE_app_cue() {}
/**
 * @brief
 *
 * @param nh
 */
void TWELITE_app_cue::Run(ros::NodeHandle *nh)
{
    if (NULL == this->publisher) {
        this->publisher = nh->advertise<std_msgs::String>(this->topic_name, 500);
    }
    if (NULL == this->timer) {
        this->timer = nh->createTimer(ros::Duration(0.1), std::bind(&TWELITE_app_cue::timer_callback, this));
    }

    this->device_open();
}
void TWELITE_app_cue::Stop()
{
    this->device_close();
}

/**
 * @brief
 *
 */
void TWELITE_app_cue::timer_callback()
{
    if (true == this->is_connected()) {
        this->device_read();
    }
}

/**
 * @brief
 *
 * @param data
 */
void TWELITE_app_cue::received(std::string data)
{
    std_msgs::String msg;
    msg.data = data;
    ROS_INFO("Publishing: '%s'", msg.data.c_str());
    this->publisher.publish(msg);
}