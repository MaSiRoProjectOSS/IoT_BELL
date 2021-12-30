
#include "SerialMonitor.h"

#include <fcntl.h>
#include <unistd.h>
void SerialMonitor::received(char *data, int size, bool indention) {}

SerialMonitor::SerialMonitor(std::string name, baudrate baud, int bits, Parity parity, int stop_bits)
{
    this->set_device_name(name);
    this->set_baud(baud);
    this->set_format(bits, parity, stop_bits);
}
void SerialMonitor::device_open(void)
{
    this->dev_fd = open(this->dev_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
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
    if (this->dev_fd >= 0) {
        this->dev_connected = true;
    } else {
        this->dev_connected = false;
    }

    /* TODO: ReadThreadを起動する */
}
void SerialMonitor::device_close(void)
{
    /* TODO: ReadThreadを停止する */
    close(this->dev_fd);

    this->dev_connected = false;
}

bool SerialMonitor::is_connected(void)
{
    return this->dev_connected;
}

void SerialMonitor::set_format(int bits, Parity parity, int stop_bits)
{
    this->dev_bits      = bits;
    this->dev_parity    = parity;
    this->dev_stop_bits = stop_bits;
}
void SerialMonitor::set_baud(baudrate baud)
{
    this->dev_baud = baud;
}
void SerialMonitor::set_device_name(std::string name)
{
    if (name.size() > 0) {
        this->dev_name = name;
    }
}
std::string SerialMonitor::get_device_name()
{
    return this->dev_name;
}

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

void SerialMonitor::device_read(void)
{
    const int   BUFFER_SIZE         = 4096;
    char        buf[BUFFER_SIZE]    = { 0 };
    static char refill[BUFFER_SIZE] = { 0 };
    static int  read_size           = 0;
    bool        flag_repeat         = true;
    bool        indention           = true;
    int         counter             = 2;
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
                    if (BUFFER_SIZE <= read_size) {
                        flag_repeat = false;
                        indention   = false;
                        break;
                    } else if ('\n' == buf[i]) {
                        flag_repeat = false;
                        indention   = true;
                        break;
                    } else if ('\r' == buf[i]) {
                        flag_repeat = false;
                        indention   = true;
                        break;
                    } else {
                        refill[read_size++] = buf[i];
                    }
                }
                if (false == flag_repeat) {
                    refill[read_size++] = '\n';
                    this->received(refill, read_size, indention);
                    if (false == indention) {
                        flag_repeat = true;
                    }
                    read_size = 0;
                } else {
                    usleep(1000);
                }
            }
        } while (true == flag_repeat);
    }
}
