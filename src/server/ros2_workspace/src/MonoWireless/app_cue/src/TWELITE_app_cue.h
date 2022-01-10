/**
 * @file TWELITE_app_cue.h
 * @brief
 * @date 2022-01-08
 *
 * @copyright Copyright (c) 2022-.
 *               MaSiRo Project.
 *
 */

#ifndef _TWELITE_APP_CUE_H_
#define _TWELITE_APP_CUE_H_

#include "ConverterAppCue.h"
#include "SerialMonitor.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>

class TWELITE_app_cue : public rclcpp::Node {
public:
    TWELITE_app_cue();

private:
    void timer_callback();

    rclcpp::Publisher<twelite_interfaces::msg::TweliteAppCueMsg>::SharedPtr publisher;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("TWELITE_app_cue");

    rclcpp::TimerBase::SharedPtr timer;
    SerialMonitor monitor;
    ConverterAppCue appcue;

    std::chrono::milliseconds tp_msec{ 250 };

    const std::string TOPIC_NAME = "TWELITE_app_cue_topic";
    rclcpp::QoS QOS{ 255 };
    const int TIMEOUT_COUNTER = (3000 / 250);
    int timeout_count         = 0;
};
#endif
