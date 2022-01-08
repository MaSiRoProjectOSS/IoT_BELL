/**
 * @file TWELITE_app_cue.h
 * @brief
 * @date 2022-01-08
 *
 * @copyright Copyright (c) 2022-.
 *               MaSiRo Project.
 *
 */

#ifndef _APP_CUE_H_
#define _APP_CUE_H_

#include "SerialMonitor.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TWELITE_app_cue : public rclcpp::Node {
public:
    TWELITE_app_cue();

private:
    void timer_callback();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr                        timer;
    SerialMonitor                                       monitor;
};
#endif
