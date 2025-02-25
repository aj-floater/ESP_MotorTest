#ifndef HM10_H
#define HM10_H

#include "mbed.h"
#include <string>
#include <cstring>

#define MAXIMUM_BUFFER_SIZE 32
class HM10 {
  private:
    // BufferedSerial object encapsulated within the HM10 class.
    BufferedSerial serial_port;
public:
    static const int BUFFER_SIZE = 32; // Adjust as needed

    // Buffers to store the last written and read data
    char currentWriteBuffer[BUFFER_SIZE];
    char currentReadBuffer[BUFFER_SIZE];

    // Constructor: initializes the serial port (9600-8-N-1)
    HM10(PinName tx, PinName rx) : serial_port(tx, rx) {
        serial_port.set_baud(9600);
        serial_port.set_format(8, BufferedSerial::None, 1);

        // Initialize the buffers
        memset(currentWriteBuffer, 0, sizeof(currentWriteBuffer));
        memset(currentReadBuffer, 0, sizeof(currentReadBuffer));
    }

    // Write function: writes the given string to the serial port and stores it in currentWriteBuffer
    void write(const char* message) {
        // Copy the message into currentWriteBuffer safely (leave space for null terminator)
        strncpy(currentWriteBuffer, message, BUFFER_SIZE - 1);
        currentWriteBuffer[BUFFER_SIZE - 1] = '\0';

        // Write the contents of currentWriteBuffer to the serial port
        serial_port.write(currentWriteBuffer, strlen(currentWriteBuffer));
    }

    // Read function: reads available data from the serial port and stores it in currentReadBuffer
    float read() {
        char tempBuffer[BUFFER_SIZE] = {0};
        uint32_t num = serial_port.read(tempBuffer, sizeof(tempBuffer) - 1);
        if (num > 0) {
            tempBuffer[num] = '\0'; // Ensure null-termination
            // Copy the received data into currentReadBuffer
            strncpy(currentReadBuffer, tempBuffer, BUFFER_SIZE - 1);
            currentReadBuffer[BUFFER_SIZE - 1] = '\0';
        }
        return num;
    }

    // Update function: placeholder for periodic tasks.
    void update() {
        // Future update logic can be added here.
    }

    // Encode data: placeholder that will output to a dynamic array of floats.
    void encodeData(const float* data, size_t size) {
        // Future encoding logic can be implemented here.
    }

    // Decode data: placeholder that takes a dynamic array of floats.
    float* decodeData(size_t& outSize) {
        // Count how many floats by counting commas.
        outSize = 1; // at least one value.
        for (size_t i = 0; i < strlen(currentReadBuffer); i++) {
            if (currentReadBuffer[i] == ',') {
                outSize++;
            }
        }
        // Allocate an array of floats.
        float* values = new float[outSize];
        size_t index = 0;

        // Use strtok to tokenize the string.
        char* token = strtok(currentReadBuffer, ",");
        while (token != NULL && index < outSize) {
            values[index++] = strtof(token, NULL);
            token = strtok(NULL, ",");
        }
        return values;
    }
};

HM10 hm10(PA_11, PA_12);

#endif // HM10_H