/**
 * @file TWELITE_AppCue.cpp
 * @brief
 * @date 2021-12-30
 *
 * @copyright Copyright (c) 2021-.
 *               MaSiRo Project.
 *
 */

#include "ConverterAppCue.h"

#include "../include/twelite_app_cue/config_twelite_app_cue.h"

// ==================================================== //
#define CONVERTERAPPCUE_LOG     0
#define CONVERTERAPPCUE_DEBUG   0
#define CONVERTERAPPCUE_VERBOSE 0
/* **************************************************** */
#include <cstdio>
#if CONVERTERAPPCUE_VERBOSE
#define verbose_printf(...) printf(__VA_ARGS__)
#else
#define verbose_printf(...)
#endif
#if CONVERTERAPPCUE_DEBUG
#define debug_printf(...) printf(__VA_ARGS__)
#else
#define debug_printf(...)
#endif
#if CONVERTERAPPCUE_LOG
#define log_printf(...) printf(__VA_ARGS__)
#else
#define log_printf(...)
#endif

/**
 * @brief Construct a new twelite app cue::twelite app cue object
 *
 */
AppCue::AppCue() {}
/**
 * @brief Destroy the twelite app cue::twelite app cue object
 *
 */
AppCue::~AppCue() {}
/**
 * @brief
 *
 * @param nh
 */
void AppCue::Run(ros::NodeHandle *nh)
{
    this->set_device_name(TWELITE::app_cue::CONFIG_TWELITE_APP_DEVICE_NAME);
    if (NULL == this->publisher) {
        this->publisher = nh->advertise<twelite_interfaces::TweliteAppCueMsg>(TWELITE::app_cue::CONFIG_TWELITE_APP_CUE_TOPIC_NAME, TWELITE::app_cue::CONFIG_TWELITE_APP_CUE_QOS);
    }
    if (NULL == this->timer) {
        this->timer = nh->createTimer(ros::Duration(0.1), std::bind(&AppCue::timer_callback, this));
    }

    this->device_open();
}
void AppCue::Stop()
{
    this->device_close();
}

/**
 * @brief
 *
 */
void AppCue::timer_callback()
{
    if (true == this->is_connected()) {
        this->device_read();
    }
}

/**
 * @brief
 *
 * @param data
 */
void AppCue::received(char *data, int size, bool indention)
{
#if CONVERTERAPPCUE_DEBUG
    ROS_DEBUG("Publishing: %s[%d] %s", indention ? ">" : "-", size, data);
#endif

    this->publisher.publish(this->convert(data, size));
}

/* **************************************************** */
// PRIVATE
/* **************************************************** */
int AppCue::convert_information(char *data, int start_index, int size, twelite_interfaces::TweliteAppCueMsg *msg)
{
    if (start_index < size) {
        msg->routers_id.data     = (data[0] << 24u) + (data[1] << 16u) + (data[2] << 8u) + (data[3]);
        msg->lqi.data            = (((7.0 * data[4]) - 1970.0) / 20.0);
        msg->seq.data            = (data[5] << 8u) + (data[6]);
        msg->end_devices_id.data = (data[7] << 24u) + (data[8] << 16u) + (data[9] << 8u) + (data[10]);
        msg->logical_id.data     = data[11];
    }
    return 11;
}
int AppCue::convert_sensor(char *data, int start_index, int size, twelite_interfaces::TweliteAppCueMsg *msg)
{
    const int TARGET_INDEX = 12;
    int result             = TARGET_INDEX + 1;
    if (start_index < size) {
        if (TARGET_INDEX < size) {
            msg->sensor.data = data[TARGET_INDEX];
            if (0x80 == data[TARGET_INDEX]) {
                // PAL
                result = this->convert_sensor_PAL(data, result, size, msg);
            } else {
                // TAG
                result = this->convert_sensor_TAG(data, result, size, msg);
            }
        }
    }
    return result;
}
int AppCue::convert_sensor_PAL(char *data, int start_index, int size, twelite_interfaces::TweliteAppCueMsg *msg)
{
    int index = start_index;
    if (start_index < size) {
        bool flag_sampling = false;
        double accel_x     = 0;
        double accel_y     = 0;
        double accel_z     = 0;
        verbose_printf("==== PAL[%d]\n", data[index]);
        msg->pal.data     = data[index] & 0x1F;
        int version       = (data[index] & 0xE0) >> 5;
        msg->version.data = ((version & 0x01) << 2) | (version & 0x02) | ((version & 0x04) >> 2);
        index++;
        int sensor_num = data[index++];
        verbose_printf("==== sensor_num[%d]\n", sensor_num);
        for (int i = 0; i < sensor_num; i++) {
            // int  __Type     = None;  // データの型を入れる
            int __ErrCode  = 0;
            int __DataType = data[index++]; // データのタイプを入れる
            int __SensorID = data[index++]; // センサのIDを入れる
            int __ExByte   = data[index++]; // 拡張バイトの内容を入れる
            int __DataNum  = data[index++]; // データのバイト数を入れる
            bool __bSigned = false;         // データが符号付かどうかを入れる
            bool __bExByte = false;         // 拡張バイトの有無を入れる
            float __Div    = 1.0;
            int __Type     = (__DataType & 0x03);
#if 0
            switch (__Type) {
                case 0:
                    verbose_printf("  --- Char\n");
                    break;
                case 1:
                    verbose_printf("  --- Short\n");
                    break;
                case 2:
                    verbose_printf("  --- Long\n");
                    break;
                default:
                    verbose_printf("  --- Variable\n");
                    break;
            }
#endif
            verbose_printf("  --- index[%d->%d] 0x%02x%02x%02x%02x\n", index, index + __DataNum, __DataType, __SensorID, __ExByte, __DataNum);
            verbose_printf("  --- ");
            for (int k = 0; k < __DataNum; k++) {
                verbose_printf("%02X", data[index + k]);
            }
            verbose_printf("\n");
            switch (__SensorID) {
                case 0x00:
                    verbose_printf(" ==== %s\n", "Magnet");
                    msg->magnet.data = data[index] & 0x7F;
                    break;
                case 0x01:
                    verbose_printf(" ==== %s\n", "Temperature");
                    __Div = 100.0;
                    // TODO : add message data
                    break;
                case 0x02:
                    verbose_printf(" ==== %s\n", "Humidity");
                    __Div = 100.0;
                    // TODO : add message data
                    break;
                case 0x03:
                    verbose_printf(" ==== %s\n", "Illuminance");
                    __Div = 100.0;
                    // TODO : add message data
                    break;
                case 0x04:
                    verbose_printf(" ==== %s\n", "Acceleration");
                    accel_x = this->byte_to_int((data[index]), (data[index + 1])) / 1000.0;
                    accel_y = this->byte_to_int((data[index + 2]), (data[index + 3])) / 1000.0;
                    accel_z = this->byte_to_int((data[index + 4]), (data[index + 5])) / 1000.0;
                    verbose_printf("      %5.2f, %5.2f, %5.2f\n", accel_x, accel_y, accel_z);

                    msg->accel.x = (msg->accel.x + accel_x) / 2.0;
                    msg->accel.y = (msg->accel.y + accel_y) / 2.0;
                    msg->accel.z = (msg->accel.z + accel_z) / 2.0;

                    if (false == flag_sampling) {
                        flag_sampling = true;
                        int freq_idx  = __ExByte >> 5;
                        switch (freq_idx) {
                            case 4:
                                msg->accel_sampling.data = 380;
                                break;
                            case 3:
                                msg->accel_sampling.data = 190;
                                break;
                            case 2:
                                msg->accel_sampling.data = 100;
                                break;
                            case 1:
                                msg->accel_sampling.data = 50;
                                break;
                            default:
                                msg->accel_sampling.data = 25;
                                break;
                        }
                    }
                    break;
                case 0x05:
                    verbose_printf(" ==== %s\n", "EventID");
                    msg->event_id.data = data[index];
                    break;
                case 0x30:
                    if (0x08 == __ExByte) {
                        verbose_printf(" ==== %s\n", "Power");
                        msg->power.data = ((data[index] << 8) + (data[index + 1]));
                    } else if (0x00 == __ExByte) {
                        debug_printf(" ==== %s\n", "ADC");
                        // msg->adc0.data = ((data[index] << 8) + (data[index + 1]));
                    } else {
                        verbose_printf(" ==== %s%d\n", "ADC", __ExByte);
                        msg->adc.data = ((data[index] << 8) + (data[index + 1]));
                    }
                    break;
                case 0x31:
                    debug_printf(" ==== %s\n", "DIO");
                    // TODO : add message data
                    break;
                case 0x32:
                    debug_printf(" ==== %s\n", "EEPROM");
                    // TODO : add message data
                    break;
                case 0x34:
                    debug_printf(" ==== %s\n", "WakeupFactor");
                    msg->packet_id.data     = data[index + 0];
                    msg->wakeup_factor.data = data[index + 1];
                    msg->wakeup_event.data  = data[index + 2];
                    break;
                default:
                    debug_printf(" ==== %s\n", "unknow");
                    break;
            }
            index += __DataNum;
        }
    }
    return index;
}
int AppCue::convert_sensor_TAG(char *data, int start_index, int size, twelite_interfaces::TweliteAppCueMsg *msg)
{
    // TODO: Set data
    if (start_index < size) {}
    return 12;
}

twelite_interfaces::TweliteAppCueMsg AppCue::convert(char *data, int size)
{
    twelite_interfaces::TweliteAppCueMsg msg;
    msg.header.frame_id = "app_cue";

    int refill_size = 0;
    char temp[0x0FFF];
    char temp1[3];
    temp1[2]        = '\0';
    bool flag_start = false;
    verbose_printf("%s() : Convert\n", __func__);

    for (int i = 0; i < size; i++) {
        if ('\n' == data[i]) {
            break;
        } else if ('\r' == data[i]) {
            break;
        } else if (':' == data[i]) {
            flag_start = true;
        } else {
            if (true == flag_start) {
                temp1[0]          = data[i];
                temp1[1]          = data[i + 1];
                temp[refill_size] = std::stoi(temp1, nullptr, 16);
                refill_size++;
                i++;
            }
        }
    }
    if (0 < refill_size) {
        int start_index = this->convert_information(temp, 0, refill_size, &msg);
        this->convert_sensor(temp, start_index, refill_size, &msg);
    }
#if CONVERTERAPPCUE_LOG
    this->DebugPrint(msg);
#endif
    return msg;
}

int AppCue::byte_to_int(char value1, char value2)
{
    int16_t buf = (value1 << 8) + (value2);
    int result  = (int)buf;
    return result;
}

/* **************************************************** */
// debug
/* **************************************************** */
void AppCue::DebugPrint(twelite_interfaces::TweliteAppCueMsg msg)
{
    log_printf("==============\n");
    log_printf("LogicalID    : %d\n", msg.logical_id.data);
    log_printf("Seq          : %d\n", msg.seq.data);
    log_printf("EndDevicesID : %08X\n", msg.end_devices_id.data);
    switch (msg.routers_id.data) {
        case 0x80000000u:
            log_printf("RoutersID    : %s\n", "No Relay");
            break;
        default:
            log_printf("RoutersID    : %08X\n", msg.routers_id.data);
            break;
    }
    switch (msg.sensor.data) {
        case msg.TWELITE_SENSOR_ANALOG:
            log_printf("Sensor       : %s\n", "Analog");
            break;
        case msg.TWELITE_SENSOR_LM61:
            log_printf("Sensor       : %s\n", "LM61");
            break;
        case msg.TWELITE_SENSOR_SHT21:
            log_printf("Sensor       : %s\n", "SHT21");
            break;
        case msg.TWELITE_SENSOR_ADT7410:
            log_printf("Sensor       : %s\n", "ADT7410");
            break;
        case msg.TWELITE_SENSOR_MPL115A2:
            log_printf("Sensor       : %s\n", "MPL115A2");
            break;
        case msg.TWELITE_SENSOR_LIS3DH:
            log_printf("Sensor       : %s\n", "LIS3DH");
            break;
        case msg.TWELITE_SENSOR_ADXL34X:
            log_printf("Sensor       : %s\n", "ADXL34x");
            break;
        case msg.TWELITE_SENSOR_TSL2561:
            log_printf("Sensor       : %s\n", "TSL2561");
            break;
        case msg.TWELITE_SENSOR_L3GD20:
            log_printf("Sensor       : %s\n", "L3GD20");
            break;
        case msg.TWELITE_SENSOR_S11059_02DT:
            log_printf("Sensor       : %s\n", "S11059-02DT");
            break;
        case msg.TWELITE_SENSOR_BME280:
            log_printf("Sensor       : %s\n", "BME280");
            break;
        case msg.TWELITE_SENSOR_SHT31:
            log_printf("Sensor       : %s\n", "SHT31");
            break;
        case msg.TWELITE_SENSOR_SHTC3:
            log_printf("Sensor       : %s\n", "SHTC3");
            break;
        case msg.TWELITE_SENSOR_ADXL362:
            log_printf("Sensor       : %s\n", "ADXL362");
            break;
        case msg.TWELITE_SENSOR_PAL:
            log_printf("Sensor       : %s\n", "PAL");
            break;
        case msg.TWELITE_SENSOR_MULTISENSOR:
            log_printf("Sensor       : %s\n", "MultiSensor");
            break;
        case msg.TWELITE_SENSOR_BUTTON:
            log_printf("Sensor       : %s\n", "Button");
            break;
        default:
            log_printf("Sensor       : %s\n", "unknow");
            break;
    }
    log_printf("     Version : %d\n", msg.version.data);
    switch (msg.pal.data) {
        case msg.TWELITE_PAL_OPEN_CLOSE_PAL:
            log_printf("PAL ID       : %s\n", "OPEN-CLOSE PAL");
            break;
        case msg.TWELITE_PAL_AMBIENT_PAL:
            log_printf("PAL ID       : %s\n", "AMBIENT PAL");
            break;
        case msg.TWELITE_PAL_MOTION_PAL:
            log_printf("PAL ID       : %s\n", "MOTION PAL");
            break;
        case msg.TWELITE_PAL_NOTICE_PAL:
            log_printf("PAL ID       : %s\n", "NOTICE PAL");
            break;
        case msg.TWELITE_PAL_TWELITE_CUE:
            log_printf("PAL ID       : %s\n", "TWELITE CUE");
            break;
        case msg.TWELITE_PAL_TWELITE_ARIA:
            log_printf("PAL ID       : %s\n", "TWELITE ARIA");
            break;
        default:
            log_printf("PAL ID       : %s\n", "unknow");
            break;
    }
    log_printf("LQI          : %5.2f [dBm]\n", msg.lqi.data);
    log_printf("Power        : %d [mV]\n", msg.power.data);
    log_printf("ADC          : %d [mV]\n", msg.adc.data);
    switch (msg.event_id.data) {
        case msg.TWELITE_EVENT_DICE_1:
            log_printf("EventID      : %s\n", "Dice1");
            break;
        case msg.TWELITE_EVENT_DICE_2:
            log_printf("EventID      : %s\n", "Dice2");
            break;
        case msg.TWELITE_EVENT_DICE_3:
            log_printf("EventID      : %s\n", "Dice3");
            break;
        case msg.TWELITE_EVENT_DICE_4:
            log_printf("EventID      : %s\n", "Dice4");
            break;
        case msg.TWELITE_EVENT_DICE_5:
            log_printf("EventID      : %s\n", "Dice5");
            break;
        case msg.TWELITE_EVENT_DICE_6:
            log_printf("EventID      : %s\n", "Dice6");
            break;
        case msg.TWELITE_EVENT_DICE_SHAKE:
            log_printf("EventID      : %s\n", "Shake");
            break;
        case msg.TWELITE_EVENT_DICE_MOVE:
            log_printf("EventID      : %s\n", "Move");
            break;
        default:
            log_printf("EventID      : %s\n", "unknow");
            break;
    }
    switch (msg.magnet.data) {
        case msg.TWELITE_MAGNET_OPEN:
            log_printf("Magnet       : %s\n", "Open");
            break;
        case msg.TWELITE_MAGNET_CLOSE_N:
            log_printf("Magnet       : %s\n", "Close(N)");
            break;
        case msg.TWELITE_MAGNET_CLOSE_S:
            log_printf("Magnet       : %s\n", "Close(S)");
            break;
        default:
            log_printf("Magnet       : %s\n", "unknow");
            break;
    }
    log_printf("WakeupFactor\n");
    log_printf("   packet id : %d\n", msg.packet_id.data);
    switch (msg.wakeup_factor.data) {
        case msg.TWELITE_WAKEUPFACTOR_SENSOR_MAGNET:
            log_printf("   factor    : %s\n", "Magnet");
            break;
        case msg.TWELITE_WAKEUPFACTOR_SENSOR_TEMPERATURE:
            log_printf("   factor    : %s\n", "Temperature");
            break;
        case msg.TWELITE_WAKEUPFACTOR_SENSOR_HUMIDITY:
            log_printf("   factor    : %s\n", "Humidity");
            break;
        case msg.TWELITE_WAKEUPFACTOR_SENSOR_ILLUMINANCE:
            log_printf("   factor    : %s\n", "Illuminance");
            break;
        case msg.TWELITE_WAKEUPFACTOR_SENSOR_ACCELERATION:
            log_printf("   factor    : %s\n", "Acceleration");
            break;
        case msg.TWELITE_WAKEUPFACTOR_SENSOR_DIO:
            log_printf("   factor    : %s\n", "DIO");
            break;
        case msg.TWELITE_WAKEUPFACTOR_SENSOR_TIMER:
            log_printf("   factor    : %s\n", "Timer");
            break;
        default:
            log_printf("   factor    : %s\n", "unknow");
            break;
    }
    switch (msg.wakeup_event.data) {
        case msg.TWELITE_WAKEUPFACTOR_EVENT_OCCURRED_EVENT:
            log_printf("   event     : %s\n", "Occurred_Event");
            break;
        case msg.TWELITE_WAKEUPFACTOR_EVENT_CHANGED_VALUE:
            log_printf("   event     : %s\n", "Changed_Value");
            break;
        case msg.TWELITE_WAKEUPFACTOR_EVENT_UPPER_THEN_THRESHOLD:
            log_printf("   event     : %s\n", "Upper_then_Threshold");
            break;
        case msg.TWELITE_WAKEUPFACTOR_EVENT_LOWER_THEN_THRESHOLD:
            log_printf("   event     : %s\n", "Lower_Then_Threshold");
            break;
        case msg.TWELITE_WAKEUPFACTOR_EVENT_WITHIN_THRESHOLD:
            log_printf("   event     : %s\n", "Within_Threshold");
            break;
        default:
            log_printf("   event     : %s\n", "unknow");
            break;
    }
    log_printf("Accel\n");
    log_printf("    X        : %5.2f [G]\n", msg.accel.x);
    log_printf("    Y        : %5.2f [G]\n", msg.accel.y);
    log_printf("    Z        : %5.2f [G]\n", msg.accel.z);
    log_printf("    Sampling : %d [Hz]\n", msg.accel_sampling.data);
}