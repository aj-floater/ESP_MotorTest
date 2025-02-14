#include "mbed.h" #

void floatToString(float value, char *buffer) {
    // Handle negative numbers
    int index = 0;
    if (value < 0) {
        buffer[index++] = '-';
        value = -value;
    }

    // Extract integer part
    int intPart = (int)value;
    float decimalPart = value - intPart;

    // Convert integer part to string (manual itoa)
    int intTemp = intPart;
    int intLen = 0;
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

    int precision = 4;

    float scale = 1;
    for (int i = 0; i < precision; i++) scale *= 10;

    // Convert fractional part (2 decimal places)
    int fracPart = (int)(decimalPart * scale + 0.5);  // Round properly

    // Convert fractional part
    for (int i = precision - 1; i >= 0; i--) {
        buffer[index + i] = '0' + (fracPart % 10);
        fracPart /= 10;
    }
    index += precision;

    // Null-terminate string
    buffer[index] = '\0';
}