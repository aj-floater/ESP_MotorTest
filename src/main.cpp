#include "mbed.h"
#include "display.h"
#include "hm10.h"

#include "potentiometer.h"

void decodingProcedure() { // >123.45,12.345,67.89
    // Call decodeData to parse the string into an array of floats.
    size_t numFloats = 0;
    float* decodedFloats = hm10.decodeData(numFloats);

    // Display
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

void encodingProcedure(){

}

int main(void)
{
    LeftPot.setRange(0.0f, 15.0f); // 0.005
    RightPot.setRange(0.0f, 0.2f); // 0.0014

    while (1) {
        LeftPot.update();
        RightPot.update();
        modify(LeftPot.getCurrentSampleMapped(), RightPot.getCurrentSampleMapped());

        display.refresh();

        if (uint32_t num = hm10.read()) {
            decodingProcedure();

            encodingProcedure();
        }
    }
}