/**
 * @file TWELITE_AppCue.cpp
 * @brief
 * @date 2021-12-30
 *
 * @copyright Copyright (c) 2021-.
 *               MaSiRo Project.
 *
 */

#include "AppCue.h"

/**
 * @brief Construct a new twelite app cue::twelite app cue object
 *
 */
AppCue::AppCue() {}
/**
 * @brief Destroy the twelite app cue::twelite app cue object
 *
 */
AppCue::~AppCue() {}
/**
 * @brief
 *
 * @param nh
 */
void AppCue::Run(ros::NodeHandle *nh)
{
    if (NULL == this->publisher) {
        this->publisher = nh->advertise<twelite_app_cue::TWELITE_app_cue_msg>(this->topic_name, 500);
    }
    if (NULL == this->timer) {
        this->timer = nh->createTimer(ros::Duration(0.1), std::bind(&AppCue::timer_callback, this));
    }

    this->device_open();
}
void AppCue::Stop()
{
    this->device_close();
}

/**
 * @brief
 *
 */
void AppCue::timer_callback()
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
void AppCue::received(char *data, int size, bool indention)
{
    //msg.data = data;
    //ROS_INFO("Publishing: '%s'[%d]", "TWELITE_app_cue", size);
    ROS_INFO("Publishing: %s[%d]'%s'", indention ? ">" : "-", size, data);

    this->publisher.publish(this->convert(data, size));
}

twelite_app_cue::TWELITE_app_cue_msg AppCue::convert(char *data, int size)
{
    twelite_app_cue::TWELITE_app_cue_msg msg;
#if 0
    int         data_size = data.size();
    const char *buffer    = data.c_str();
    char        temp[0x0FFF];
    int         index = 0;
    char        temp1[2];

    for (int i = 0; i < data_size; i++) {
        if ('\n' == buffer[i]) {
            break;
        } else if ('\r' == buffer[i]) {
            break;
        } else if (':' == buffer[i]) {
        } else {
            temp1[0] = buffer[i];
            temp1[1] = buffer[i + 1];
            //            temp[index] = std::stoi(temp1, nullptr, 16);
            printf("%c%c", temp1[0], temp1[1]);
            index++;
            i++;
        }
    }
    for (int j = 0; j < index; j++) {
        //     printf("%x,", temp[j]);
    }
    printf("\n\n\n");
#endif

    return msg;
}