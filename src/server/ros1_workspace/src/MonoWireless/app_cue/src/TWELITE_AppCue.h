
#ifndef _APP_CUE_H_
#define _APP_CUE_H_

#include "SerialMonitor.h"

#include <ros/ros.h>
#include <std_msgs/String.h>

class TWELITE_app_cue : public SerialMonitor {
public:
    TWELITE_app_cue();
    ~TWELITE_app_cue();

    void Run(ros::NodeHandle *nh);
    void Stop();

private:
    void timer_callback();
    void received(std::string data);

    const std::string topic_name = "TWELITE_app_cue";
    ros::Publisher    publisher;
    ros::Timer        timer;
};
#endif
