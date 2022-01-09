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

// ==================================================== //
#define TWELITE_APP_CUE_MAIN_DEBUG 1
/* **************************************************** */
#if TWELITE_APP_CUE_MAIN_DEBUG
#define debug_printf(...) printf(__VA_ARGS__)
#else
#define debug_printf(...)
#endif
#define log_printf(...) printf(__VA_ARGS__)
// ==================================================== //

int main(int argc, char *argv[])
{
    log_printf(" === TWELITE_app_cue : start ===\n");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TWELITE_app_cue>());
    rclcpp::shutdown();
    log_printf(" === TWELITE_app_cue : shutdown ===\n");
    return 0;
}
