/**
 * @file TWELITE_app_cue.cpp
 * @brief
 * @date 2022-01-08
 *
 * @copyright Copyright (c) 2022-.
 *               MaSiRo Project.
 *
 */
#include "TWELITE_app_cue.h"

#include "SerialMonitor.h"

#include <functional>
#include <memory>
#include <string>

// ==================================================== //
#define TWELITE_APP_CUE_DEBUG 0
/* **************************************************** */
#if TWELITE_APP_CUE_DEBUG
#define debug_printf(...) printf(__VA_ARGS__)
#else
#define debug_printf(...)
#endif
#define log_printf(...) printf(__VA_ARGS__)
// ==================================================== //

TWELITE_app_cue::TWELITE_app_cue() : Node("node_TWELITE_app_cue")
{
    this->publisher = this->create_publisher<std_msgs::msg::String>(this->TOPIC_NAME, this->BUFFER_SIZE);
    this->timer     = this->create_wall_timer(this->tp_msec, std::bind(&TWELITE_app_cue::timer_callback, this));
}

void TWELITE_app_cue::timer_callback()
{
    if (true == this->monitor.is_connected()) {
        this->counter = this->TIMEOUT_COUNTER;
        std::string outdata;
        bool one_sentence;
        if (true == this->monitor.device_read(&outdata, &one_sentence)) {
            if (true == one_sentence) {
                debug_printf("%s() : received [%ld][%s]\n", __func__, outdata.size(), outdata.c_str());
                twelite_interfaces::msg::TweliteAppCueMsg msg = this->appcue.Convert(outdata.c_str(), outdata.size());
            }
        }
    } else {
        if (0 <= this->counter) {
            this->counter = this->TIMEOUT_COUNTER;
            this->monitor.device_close();
            this->monitor.device_open();
            debug_printf("%s : device open\n", TWELITE_app_cue);
        } else {
            this->counter--;
            debug_printf("%s : device missing\n", TWELITE_app_cue);
        }
    }
}
