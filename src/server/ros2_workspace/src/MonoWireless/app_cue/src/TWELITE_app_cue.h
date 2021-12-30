
#ifndef _APP_CUE_H_
#define _APP_CUE_H_

#include "serial_monitor.h"

class TWELITE_app_cue : SerialMonitor, rclcpp::Node {
public:
    TWELITE_app_cue() : Node("node_TWELITE_app_cue")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("TWELITE_app_cue_topic", 10);
        timer_     = this->create_wall_timer(500ms, std::bind(&TWELITE_app_cue::timer_callback, this));
    }

private:
    void timer_callback()
    {
     this->   device_read();
    }
    void received(std::string data)
    {
        auto message = std_msgs::msg::String();
        message.data = data;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr                        timer_;
};
#endif
