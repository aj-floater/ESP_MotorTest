#include "mbed.h"
#include "display.h"
#include "hm10.h"

int main(void)
{

    while (1) {
        if (uint32_t num = hm10.read()) {

            // Call decodeData to parse the string into an array of floats.
            size_t numFloats = 0;
            float* decodedFloats = hm10.decodeData(numFloats);

            // Echo the input to the display plus -group21 string.
            // char output[MAXIMUM_BUFFER_SIZE + 10] = {0}; // Extra space for "group21"
            // snprintf(output, sizeof(output), "%s%s", hm10.currentReadBuffer, "-group21");
            // hm10.write(output);
            // display.refresh();
            char buffer[50];
            display.lcd.locate(0,0);
            floatToString(decodedFloats[0], buffer);
            display.lcd.printf(buffer);
            display.lcd.locate(0,10);
            floatToString(decodedFloats[1], buffer);
            display.lcd.printf(buffer);
            display.lcd.locate(0,20);
            floatToString(decodedFloats[2], buffer);
            display.lcd.printf(buffer);
        }
    }
}