/**
 * @file main.cpp
 * @brief TWELITE_app_cueのmain関数
 * @date 2022-01-08
 *
 * @copyright Copyright (c) 2022-.
 *               MaSiRo Project.
 *
 */

#include "TWELITE_app_cue.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TWELITE_app_cue>());
    rclcpp::shutdown();
    return 0;
}
