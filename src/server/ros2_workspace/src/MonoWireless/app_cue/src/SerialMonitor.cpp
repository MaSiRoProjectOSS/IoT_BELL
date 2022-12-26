/**
 * @file SerialMonitor.cpp
 * @brief SerialMonitor for Linux
 * @date 2022-01-08
 *
 * @copyright Copyright (c) 2022-.
 *               MaSiRo Project.
 *
 */

#include "SerialMonitor.h"

#include <fcntl.h>
#include <unistd.h>
// ==================================================== //
#define SERIALMONITOR_DEBUG 0
/* **************************************************** */
#if SERIALMONITOR_DEBUG
#define debug_printf(...) printf(__VA_ARGS__)
#else
#define debug_printf(...)
#endif
#define log_printf(...) printf(__VA_ARGS__)
// ==================================================== //

SerialMonitor::SerialMonitor(std::string name, Baudrate baud, int bits, Parity parity, int stop_bits)
{
    this->set_device_name(name);
    this->set_baud(baud);
    this->set_format(bits, parity, stop_bits);
}

//////////////////////////////////////////////////////////
// 設定 API
//////////////////////////////////////////////////////////
#pragma region SETTING_API
void SerialMonitor::set_device_name(std::string name)
{
    if (name.size() > 0) {
        this->dev_name = name;
    }
}

void SerialMonitor::set_baud(Baudrate baud)
{
    this->dev_baud = baud;
}

void SerialMonitor::set_format(int bits, Parity parity, int stop_bits)
{
    this->dev_bits      = bits;
    this->dev_parity    = parity;
    this->dev_stop_bits = stop_bits;
}

std::string SerialMonitor::get_device_name()
{
    return this->dev_name;
}
#pragma endregion

//////////////////////////////////////////////////////////
// デバイス接続 API
//////////////////////////////////////////////////////////
#pragma region CONNECT_API
void SerialMonitor::device_open(void)
{
    if (-1 == this->dev_fd) {
        this->dev_fd = open(this->dev_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (this->dev_fd >= 0) {
            this->dev_connected = true;
            fcntl(this->dev_fd, F_SETFL, 0);
            //load configuration
            struct termios conf_tio;
            tcgetattr(this->dev_fd, &conf_tio);
            //set baudrate
            speed_t BAUDRATE = this->dev_baud;
            cfsetispeed(&conf_tio, BAUDRATE);
            //non canonical, non echo back
            conf_tio.c_lflag &= ~(ECHO | ICANON);
            //non blocking
            conf_tio.c_cc[VMIN]  = 0;
            conf_tio.c_cc[VTIME] = 0;
            //store configuration
            tcsetattr(this->dev_fd, TCSANOW, &conf_tio);

            debug_printf("%s()\n", __func__);
            log_printf("open device : \"%s\" %s/bps/data %d bit/%s/stop %d bit\n",
                       this->dev_name.c_str(),
                       this->textBaudrate(this->dev_baud).c_str(),
                       this->dev_bits,
                       this->textParity(this->dev_parity).c_str(),
                       this->dev_stop_bits);
        } else {
            this->dev_connected = false;
        }
    }
}

bool SerialMonitor::is_connected(void)
{
    return this->dev_connected;
}

void SerialMonitor::device_close(void)
{
    if (this->dev_fd >= 0) {
        close(this->dev_fd);
        this->dev_fd = -1;
    }
    debug_printf("%s()\n", __func__);

    this->dev_connected = false;
}
#pragma endregion

//////////////////////////////////////////////////////////
// 操作 API
//////////////////////////////////////////////////////////
#pragma region OPERATION_API
int SerialMonitor::device_write(std::string data)
{
    int result = 0;
    if (this->dev_connected) {
        result = write(this->dev_fd, data.c_str(), data.size());
        if (result < 0) {
            this->dev_connected = false;
        }
    }
    return result;
}

bool SerialMonitor::device_read(std::string *outdata, bool *one_sentence)
{
    bool result                     = false;
    const int BUFFER_SIZE           = 4096;
    char buf[BUFFER_SIZE]           = { 0 };
    static char refill[BUFFER_SIZE] = { 0 };
    static int read_size            = 0;
    bool flag_repeat                = true;
    int counter                     = 2;
    *outdata                        = "";
    if (true == this->dev_connected) {
        do {
            int recv_data = read(this->dev_fd, buf, sizeof(buf));
            if (recv_data <= 0) {
                if (0 >= counter) {
                    flag_repeat = false;
                }
                counter--;
                break;
            } else {
                for (int i = 0; i < recv_data; i++) {
                    if ((BUFFER_SIZE - 2) <= read_size) {
                        flag_repeat   = false;
                        *one_sentence = false;
                        break;
                    } else if ('\n' == buf[i]) {
                        flag_repeat   = false;
                        *one_sentence = true;
                        break;
                    } else if ('\r' == buf[i]) {
                        flag_repeat   = false;
                        *one_sentence = true;
                        break;
                    } else {
                        refill[read_size++] = buf[i];
                    }
                }
                if (false == flag_repeat) {
                    if (0 != read_size) {
                        refill[read_size++] = '\n';
                        refill[read_size]   = '\0';
                        *outdata = refill;
                        result   = true;
                    }
                    if (false == *one_sentence) {
                        flag_repeat = true;
                    }
                    read_size = 0;
                } else {
                    usleep(1000);
                }
            }
        } while (true == flag_repeat);
    }
    return result;
}
#pragma endregion

/* **************************************************** */
//
/* **************************************************** */
#pragma region PRIVATE_API

std::string SerialMonitor::textParity(Parity parity)
{
    switch (parity) {
        case Parity::None:
            return "None";
        case Parity::Odd:
            return "Odd";
        case Parity::Even:
            return "Even";
        case Parity::Forced1:
            return "Forced1";
        case Parity::Forced0:
            return "Forced0";
        default:
            return "unknow";
    }
}
std::string SerialMonitor::textBaudrate(Baudrate baud)
{
    switch (baud) {
        case Baudrate::BaudRate_57600:
            return "57,600";
        case Baudrate::BaudRate_115200:
            return "115,200";
        case Baudrate::BaudRate_230400:
            return "230,400";
        case Baudrate::BaudRate_460800:
            return "460,800";
        case Baudrate::BaudRate_500000:
            return "500,000";
        case Baudrate::BaudRate_576000:
            return "576,000";
        case Baudrate::BaudRate_921600:
            return "921,600";
        default:
            return "---";
    }
}
#pragma endregion
