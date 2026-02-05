#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <stdexcept>

class SerialPortException : public std::runtime_error {
public:
    SerialPortException(const std::string& message);
};

class SerialPort {
public:
    SerialPort();
    SerialPort(const char* port_name);
    ~SerialPort();

    void open();
    void configure(unsigned int baud_rate, int data_bits = 8, char parity = 'N', int stop_bits = 1);
    void write(const char* data, int length);
    void read(char* buffer, int buffer_size);
    void close();
    void set_port_name(const char* port_name);

private:
    const char* port_name_;
    int fd_;
};

#endif // SERIAL_PORT_H

