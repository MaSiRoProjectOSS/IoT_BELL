/**
 * @file SerialMonitor.h
 * @brief SerialMonitor for Linux
 * @date 2022-01-08
 *
 * @copyright Copyright (c) 2022-.
 *               MaSiRo Project.
 *
 */

#ifndef _SERIAL_MONITOR_H_
#define _SERIAL_MONITOR_H_

#include <chrono>
#include <memory>
#include <termios.h>

class SerialMonitor {
public:
    /**
     * @brief 	The parity of the serial port.
     */
    enum Parity
    {
        None,    /* パリティなし */
        Odd,     /* 奇数パリティ */
        Even,    /* 偶数パリティ */
        Forced1, /* 1強制 */
        Forced0  /* 0強制 */
    };
    /**
     *@brief The baud rate of the serial port.
     */
    enum Baudrate
    {
        BaudRate_57600  = B57600,
        BaudRate_115200 = B115200,
        BaudRate_230400 = B230400,
        BaudRate_460800 = B460800,
        BaudRate_500000 = B500000,
        BaudRate_576000 = B576000,
        BaudRate_921600 = B921600,
    };

public:
    /**
     * @brief Construct a new Serial Monitor object
     *
     * @param name      The device name(default = /dev/ttyUSB0)
     * @param baud      The baud rate
     * @param bits      The number of bits in a word (5-8; default = 8)
     * @param parity    The parity used (None, Odd, Even, Forced1, Forced0; default = None)
     * @param stop_bits The number of stop bits (1 or 2; default = 1)
     */
    SerialMonitor(std::string name = "/dev/ttyUSB0", Baudrate baud = BaudRate_115200, int bits = 8, Parity parity = Parity::None, int stop_bits = 1);

    //////////////////////////////////////////////////////////
    // 設定 API
    //////////////////////////////////////////////////////////
    void set_device_name(std::string name);
    void set_baud(Baudrate baud);
    void set_format(int bits = 8, Parity parity = Parity::None, int stop_bits = 1);

    std::string get_device_name();

    //////////////////////////////////////////////////////////
    // デバイス接続 API
    //////////////////////////////////////////////////////////
    void device_open(void);
    bool is_connected(void);
    void device_close(void);

    //////////////////////////////////////////////////////////
    // 操作 API
    //////////////////////////////////////////////////////////
    int device_write(std::string data);
    bool device_read(std::string *outdata, bool *one_sentence);

private:
    /* **************************************************** */
    //
    /* **************************************************** */
    std::string textParity(Parity parity);
    std::string textBaudrate(Baudrate baud);

    /* **************************************************** */
    // 設定
    /* **************************************************** */
    std::string dev_name;
    Parity dev_parity;
    bool dev_connected;
    int dev_bits;
    int dev_stop_bits;
    Baudrate dev_baud;

    /* **************************************************** */
    //
    /* **************************************************** */
    char buffer_data[255] = { 0 };
    int buffer_index      = 0;

    /* **************************************************** */
    //
    /* **************************************************** */
    int dev_fd = -1;
};

#endif