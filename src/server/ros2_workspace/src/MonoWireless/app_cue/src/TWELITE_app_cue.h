/**
 * @file TWELITE_app_cue.h
 * @brief TWELITE® CUE converter
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

    rclcpp::TimerBase::SharedPtr timer;
    SerialMonitor monitor;
    ConverterAppCue appcue;

    std::chrono::milliseconds tp_msec{ 250 };

    const int TIMEOUT_COUNTER = (3000 / 250);
    int timeout_count         = 0;
};
#endif
