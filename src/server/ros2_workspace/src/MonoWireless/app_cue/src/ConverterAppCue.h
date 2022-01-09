/**
 * @file ConverterAppCue.h
 * @brief Convert AppCue data
 * @date 2022-01-09
 *
 * @copyright Copyright (c) 2022-.
 *               MaSiRo Project.
 *
 */
#ifndef _CONVERTOR_APP_CUE_H_
#define _CONVERTOR_APP_CUE_H_

#include "rclcpp/rclcpp.hpp"
#include "twelite_interfaces/msg/twelite_app_cue_msg.hpp"

class ConverterAppCue {
public:
    ConverterAppCue();
    ~ConverterAppCue();

    twelite_interfaces::msg::TweliteAppCueMsg Convert(const char *data, int size);

private:
    /* **************************************************** */
    // converter
    /* **************************************************** */
    int convert_information(char *data, int start_index, int size, twelite_interfaces::msg::TweliteAppCueMsg *msg);
    int convert_sensor(char *data, int start_index, int size, twelite_interfaces::msg::TweliteAppCueMsg *msg);
    int convert_sensor_PAL(char *data, int start_index, int size, twelite_interfaces::msg::TweliteAppCueMsg *msg);
    int convert_sensor_TAG(char *data, int start_index, int size, twelite_interfaces::msg::TweliteAppCueMsg *msg);

    /* **************************************************** */
    // debug
    /* **************************************************** */
    void DebugPrint(twelite_interfaces::msg::TweliteAppCueMsg msg);
};
#endif
