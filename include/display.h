#ifndef DISPLAY_H
#define DISPLAY_H

#include "mbed.h"
#include "C12832.h"

#include "wheel.h"

#define MAXIMUM_BUFFER_SIZE                                                  32
// Application buffer to receive the data
char buf[MAXIMUM_BUFFER_SIZE] = {0};

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

class Display {
private:
    C12832 lcd;
    bool screenNeedsRefresh;
    char buffer[50];  // Buffer for number-to-string conversion

public:
    // Constructor
    Display(PinName mosi, PinName sck, PinName reset, PinName a0, PinName ncs)
        : lcd(mosi, sck, reset, a0, ncs), screenNeedsRefresh(false) {}

    // Method to mark display for refresh
    void markRefreshNeeded() {
        screenNeedsRefresh = true;
    }

    // Method to refresh display
    void refresh() {
        if (screenNeedsRefresh) {
            lcd.cls();
            screenNeedsRefresh = false;
        }

        lcd.locate(0, 0);
        lcd.printf("buf: %s\n", buf);

        // floatToString(right_wheel.proportional_gain, buffer);
        // lcd.printf("Kp: %s\n", buffer);
        
        // lcd.locate(80, 0);
        // floatToString(right_wheel.desired_speed, buffer);
        // lcd.printf("rs: %s\n", buffer);
        // // floatToString(right_wheel.integral_gain, buffer);
        // // lcd.printf("Ki: %s\n", buffer);
        // // floatToString(right_wheel.derivative_gain, buffer);
        // // lcd.printf("Kd: %s\n", buffer);

        // lcd.locate(0, 10);
        // floatToString(deadband, buffer);
        // lcd.printf("db: %s\n", buffer);

        // lcd.locate(80, 10);
        // floatToString(minScaleFactor, buffer);
        // lcd.printf("mSF: %s\n", buffer);

        // lcd.locate(0, 20);
        // floatToString(Kp, buffer);
        // lcd.printf("kp: %s\n", buffer);

        // lcd.locate(80, 20);
        // floatToString(Kp_straight, buffer);
        // lcd.printf("kps: %s\n", buffer);
    }
};

Display display(D11, D13, D12, D7, D10);

#endif // DISPLAY_H