#include "mbed.h"

#include "hm10.h"

#include "potentiometer.h"
#include "input.h"
#include "encoder.h"
#include "wheel.h"
#include "display.h"

void encodingProcedure(){
    float encodingFloats[4] = {
        left_wheel.measured_speed_angular(),
        left_wheel.desired_speed,
        right_wheel.measured_speed_angular(),
        right_wheel.desired_speed
    };

    hm10.encodeData(encodingFloats, 4);
    hm10.write();
}

int main(void)
{
    // Pin Setup
    // ----------------------------
    DigitalOut Bipolar1(PB_13);
    Bipolar1.write(0);
    // DigitalOut Direction1(PB_14);
    // Direction1.write(1);
    // PwmOut Motor1(PC_6);
    DigitalOut Bipolar2(PB_15);
    Bipolar2.write(0);
    // DigitalOut Direction2(PB_1);
    // Direction2.write(1);
    // PwmOut Motor2(PC_8);
    DigitalOut Enable(PB_2);
    Enable.write(1);
    // ----------------------------

    LeftPot.setRange(0.0f, 15.0f); // 0.005
    RightPot.setRange(0.0f, 0.2f); // 0.0014

    right_wheel.speed(0.0f);
    left_wheel.speed(0.0f);

    while (1) {
        LeftPot.update();
        RightPot.update();
        // modify(LeftPot.getCurrentSampleMapped(), RightPot.getCurrentSampleMapped());

        display.refresh();
        
        // Bluetooth code starts here

        // this has to be called constantly during the loop, it's not an interrupt
        if (hm10.read() != -EAGAIN) { // check for new data from the hm10 module
            printf(hm10.currentReadBuffer); // this hm10.currentReadBuffer is filled with data whenever hm10.read() is called
            // here we are just printing it to the serial, but you could test if it's 0 or 1 etc to change 

            // what i've found is that sticking a random character before the character you want to be read is the only
            // way to read reliably (eg send '>0' to read '0')
        }
    }
}