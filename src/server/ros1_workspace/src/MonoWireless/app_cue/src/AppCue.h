
#ifndef _APP_CUE_H_
#define _APP_CUE_H_

#include "SerialMonitor.h"
#include "twelite_app_cue/TWELITE_app_cue_msg.h"

#include <ros/ros.h>

class AppCue : public SerialMonitor {
public:
    AppCue();
    ~AppCue();

    void Run(ros::NodeHandle *nh);
    void Stop();

private:
    void                                 timer_callback();
    void                                 received(char *data, int size, bool indention);
    twelite_app_cue::TWELITE_app_cue_msg convert(char *data, int size);

    const std::string topic_name = "TWELITE_app_cue";
    ros::Publisher    publisher;
    ros::Timer        timer;
};
#endif
