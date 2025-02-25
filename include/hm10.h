#ifndef HM10_H
#define HM10_H

#include "mbed.h"
#include <string>
#include <cstring>

class HM10 {
  private:
    // BufferedSerial object encapsulated within the HM10 class.
    BufferedSerial serial_port;
public:
    // Public buffers to store the most recent write and read strings.
    std::string currentWriteBuffer;
    std::string currentReadBuffer;

    // Constructor: Initialize the serial port with 9600-8-N-1.
    HM10(PinName tx, PinName rx) : serial_port(tx, rx) {
        serial_port.set_baud(9600);
        serial_port.set_format(8, BufferedSerial::None, 1);
    }

    // Send function: writes the given string to the serial port and stores it.
    void send(const char* message) {
        currentWriteBuffer = message;
        serial_port.write(message, strlen(message));
    }

    // Read function: reads available data from the serial port and stores it.
    void read() {
        char buf[128] = {0};  // Adjust size as needed.
        uint32_t num = serial_port.read(buf, sizeof(buf) - 1);
        if (num > 0) {
            buf[num] = '\0';  // Ensure null-termination.
            currentReadBuffer = std::string(buf);
        }
    }

    // Update function: placeholder for periodic tasks.
    void update() {
        // Future update logic can be added here.
    }

    // Encode data: placeholder that takes a dynamic array of floats.
    void encodeData(const float* data, size_t size) {
        // Future encoding logic can be implemented here.
    }

    // Decode data: placeholder that takes a dynamic array of floats.
    void decodeData(const float* data, size_t size) {
        // Future decoding logic can be implemented here.
    }
};

#endif // HM10_H