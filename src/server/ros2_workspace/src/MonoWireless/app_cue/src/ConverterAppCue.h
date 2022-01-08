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

//#include "twelite_app_cue/TWELITE_app_cue_msg.h"

class ConverterAppCue {
public:
    ConverterAppCue();
    ~ConverterAppCue();

#if 0
    twelite_app_cue::TweliteAppCueMsg Convert(char *data, int size);

private:
    /* **************************************************** */
    // converter
    /* **************************************************** */
    int convert_information(char *data, int start_index, int size, twelite_app_cue::TWELITE_app_cue_msg *msg);
    int convert_sensor(char *data, int start_index, int size, twelite_app_cue::TWELITE_app_cue_msg *msg);
    int convert_sensor_PAL(char *data, int start_index, int size, twelite_app_cue::TWELITE_app_cue_msg *msg);
    int convert_sensor_TAG(char *data, int start_index, int size, twelite_app_cue::TWELITE_app_cue_msg *msg);

    /* **************************************************** */
    // debug
    /* **************************************************** */
    void DebugPrint(twelite_app_cue::TWELITE_app_cue_msg msg);
#endif
};
#endif
