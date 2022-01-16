
#ifndef _APP_CUE_H_
#define _APP_CUE_H_

#include "SerialMonitor.h"
#include "twelite_interfaces/TweliteAppCueMsg.h"

#include <ros/ros.h>

class AppCue : public SerialMonitor {
public:
    AppCue();
    ~AppCue();

    void Run(ros::NodeHandle *nh);
    void Stop();

private:
    void timer_callback();
    void received(char *data, int size, bool indention);
    twelite_interfaces::TweliteAppCueMsg convert(char *data, int size);

    /* **************************************************** */
    // converter
    /* **************************************************** */
    int convert_information(char *data, int start_index, int size, twelite_interfaces::TweliteAppCueMsg *msg);
    int convert_sensor(char *data, int start_index, int size, twelite_interfaces::TweliteAppCueMsg *msg);
    int convert_sensor_PAL(char *data, int start_index, int size, twelite_interfaces::TweliteAppCueMsg *msg);
    int convert_sensor_TAG(char *data, int start_index, int size, twelite_interfaces::TweliteAppCueMsg *msg);

    int byte_to_int(char value1, char value2);
    /* **************************************************** */
    // debug
    /* **************************************************** */
    void DebugPrint(twelite_interfaces::TweliteAppCueMsg msg);

    ros::Publisher publisher;
    ros::Timer timer;
};
#endif
