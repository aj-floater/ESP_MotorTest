#include "mbed.h"
#include "display.h"
#include "hm10.h"

#include "potentiometer.h"

DigitalOut FLASH_LED(D9);

void decodingProcedure() { // >123.45,12.345,67.89
    // Call decodeData to parse the string into an array of floats.
    size_t numFloats = 0;
    float* decodedFloats = hm10.decodeData(numFloats);

    if (decodedFloats[0] >= 180){
        FLASH_LED = 0;
    } else {
        FLASH_LED = 1;
    }

    // if (decodedFloats[0] >= 255) decodedFloats[0] = 0;
    // if (decodedFloats[1] >= 255) decodedFloats[1] = 0;

    // Display
    char buffer[50];
    display.lcd.locate(0,0);
    floatToString(decodedFloats[0], buffer);
    display.lcd.printf(buffer);
    display.lcd.locate(0,10);
    floatToString(decodedFloats[1], buffer);
    display.lcd.printf(buffer);
    // display.lcd.locate(0,20);
    // floatToString(decodedFloats[2], buffer);
    // display.lcd.printf(buffer);
}

void encodingProcedure(){
    float encodingFloats[2] = {LeftPot.getCurrentSampleMapped(), RightPot.getCurrentSampleMapped()};

    hm10.encodeData(encodingFloats, 2);
    hm10.write();
}

int main(void)
{
    FLASH_LED = 1.0f;

    LeftPot.setRange(0.0f, 15.0f); // 0.005
    RightPot.setRange(0.0f, 0.2f); // 0.0014

    while (1) {
        LeftPot.update();
        RightPot.update();
        modify(LeftPot.getCurrentSampleMapped(), RightPot.getCurrentSampleMapped());

        display.refresh();
        encodingProcedure();

        if (uint32_t num = hm10.read()) {
            // FLASH_LED = !FLASH_LED;
            decodingProcedure();

            // encodingProcedure();
        }
    }
}