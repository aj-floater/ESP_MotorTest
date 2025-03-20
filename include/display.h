#ifndef DISPLAY_H
#define DISPLAY_H

#include "mbed.h"


float deadband = 0.05f;
float Kp = 9.0f;             // Proportional gain - tune as needed
float Kp_straight = 2.35f;  // Tune this parameter
float minScaleFactor = 0.8f; // At least 80% of speed, tune as needed
float i = 0;
float a = 0;

void floatToString(float value, char *buffer);

// Helper function to convert float to string
void floatToString(float value, char *buffer) {
    int index = 0;

    // Handle negative numbers
    if (value < 0) {
        buffer[index++] = '-';
        value = -value;
    }

    // Extract integer and fractional parts
    int intPart = static_cast<int>(value);
    float decimalPart = value - intPart;

    // Convert integer part to string
    int intTemp = intPart, intLen = 0;
    do {
        intLen++;
        intTemp /= 10;
    } while (intTemp > 0);

    for (int i = intLen - 1; i >= 0; i--) {
        buffer[index + i] = '0' + (intPart % 10);
        intPart /= 10;
    }
    index += intLen;

    // Add decimal point
    buffer[index++] = '.';

    // Convert fractional part (4 decimal places)
    int precision = 4;
    float scale = 1;
    for (int i = 0; i < precision; i++) scale *= 10;

    int fracPart = static_cast<int>(decimalPart * scale + 0.5);  // Proper rounding
    for (int i = precision - 1; i >= 0; i--) {
        buffer[index + i] = '0' + (fracPart % 10);
        fracPart /= 10;
    }
    index += precision;

    // Null-terminate string
    buffer[index] = '\0';
}


#endif // DISPLAY_H