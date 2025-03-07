#ifndef HM10_H
#define HM10_H

#include "mbed.h"

#include "potentiometer.h"

#include <string>
#include <cstring>

void floatToString(float value, char *buffer);

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
        serial_port.set_blocking(false);

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

    // Write function: writes the given string to the serial port and stores it in currentWriteBuffer
    void write() {
        // Write the contents of currentWriteBuffer to the serial port
        serial_port.write(currentWriteBuffer, strlen(currentWriteBuffer));
    }

    // Read function: reads available data from the serial port and stores it in currentReadBuffer
    float read() {
        char tempBuffer[BUFFER_SIZE] = {0};
        uint32_t num = serial_port.read(tempBuffer, sizeof(tempBuffer) - 1);
        // Check for -EAGAIN (i.e., no data available in non-blocking mode)
        if (num == -EAGAIN) {
            // Return immediately if no data is available.
            // currentReadBuffer = 
            return -EAGAIN;
        }
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

    // Encode a dynamic array of floats into a comma-separated string.
    // Save that encoded data into the currentWriteBuffer
    void encodeData(const float* data, size_t size) {
        // Clear the buffer.
        memset(currentWriteBuffer, 0, BUFFER_SIZE);
        char temp[16]; // Temporary buffer for each float conversion.
        for (size_t i = 0; i < size; i++) {
            // Convert each float to string; adjust format as needed.
            floatToString(data[i], temp);
            strncat(currentWriteBuffer, temp, BUFFER_SIZE - strlen(currentWriteBuffer) - 1);
            if (i < size - 1) {
                // Append a comma if not the last element.
                strncat(currentWriteBuffer, ",", BUFFER_SIZE - strlen(currentWriteBuffer) - 1);
            }
        }
        // currentWriteBuffer now contains the encoded data.
    }

    // Decode a comma-separated string of floats from currentReadBuffer.
    // The outSize parameter will be set to the number of floats.
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

class Controller {
public:
    float leftJoystickY = 0.0f;
    float rightJoystickX = 0.0f;

    // Maps [0..128] to [1..0], and [128..255] to [0..-1].
    float mapJoystickValue(float val)
    {
        // Clamp just to be safe if something goes out of range:
        if (val < 0.0f)   val = 0.0f;
        if (val > 255.0f) val = 255.0f;

        if (val < 128.0f)
        {
            // Scale [0..128] -> [1..0]
            return 1.0f - (val / 128.0f);
        }
        else if (val == 128.0f){
            return 0.0f;
        }
        else
        {
            // Scale (128..255] -> (0..-1]
            // 255 - 128 = 127, so we divide by 127 for that half
            return -((val - 128.0f) / 127.0f);
        }
    }
};

Controller stadia;


void decodingProcedure() { // >123.45,12.345,67.89
    // Call decodeData to parse the string into an array of floats.
    size_t numFloats = 0;
    float* decodedFloats = hm10.decodeData(numFloats);

    stadia.leftJoystickY  = stadia.mapJoystickValue(decodedFloats[0]);
    stadia.rightJoystickX = stadia.mapJoystickValue(decodedFloats[1]);
    
    // Display
    char buffer1[50];
    char buffer2[50];
    floatToString(decodedFloats[0], buffer1);
    floatToString(decodedFloats[1], buffer2);
    // printf("readbuffer: %s. values: %s, %s\n", hm10.currentReadBuffer, buffer1, buffer2);
}

#endif // HM10_H