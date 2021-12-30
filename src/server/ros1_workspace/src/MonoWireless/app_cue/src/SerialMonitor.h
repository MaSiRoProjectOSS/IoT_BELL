
#ifndef _SERIAL_MONITOR_H_
#define _SERIAL_MONITOR_H_

#include <chrono>
#include <memory>
#include <termios.h>

class SerialMonitor {
public:
    enum Parity
    {
        None,    /* パリティなし */
        Odd,     /* 奇数パリティ */
        Even,    /* 偶数パリティ */
        Forced1, /* 1強制 */
        Forced0  /* 0強制 */
    };
    enum baudrate
    {
        BaudRate_57K_600  = B57600,
        BaudRate_115K_200 = B115200,
        BaudRate_23K_400  = B230400,
        BaudRate_460K_800 = B460800,
        BaudRate_500K     = B500000,
        BaudRate_576K     = B576000,
        BaudRate_921_K600 = B921600,
        BaudRate_1M       = B1000000,
        BaudRate_1M_152K  = B1152000,
        BaudRate_1M_500K  = B1500000,
        BaudRate_2M       = B2000000,
        BaudRate_2M_500K  = B2500000,
        BaudRate_3M       = B3000000,
        BaudRate_3M_500K  = B3500000,
        BaudRate_4M       = B4000000,
    };

public:
    SerialMonitor(std::string name = "/dev/ttyUSB0", baudrate baud = BaudRate_1M_152K, int bits = 8, Parity parity = Parity::None, int stop_bits = 1);

    void device_open(void);
    bool is_connected(void);
    void device_close(void);

    void set_format(int bits = 8, Parity parity = Parity::None, int stop_bits = 1);
    void set_baud(baudrate baud);
    void set_device_name(std::string name);

    std::string get_device_name();

    int  device_write(std::string data);
    void device_read(void);

    void monitor(void);

private:
    virtual void received(std::string data);

    int dev_fd = -1;

    std::string dev_name;
    Parity      dev_parity;
    bool        dev_connected;
    int         dev_bits;
    int         dev_stop_bits;
    baudrate    dev_baud;
    char        buffer_data[255] = { 0 };
    int         buffer_index     = 0;
};

#endif