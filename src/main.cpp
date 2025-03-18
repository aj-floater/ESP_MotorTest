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

        encodingProcedure();
        if (hm10.read() != -EAGAIN) {
            // size_t numFloats = 0;
            // float* decodedFloats = hm10.decodeData(numFloats);

            // hm10.read();
            
            display.lcd.cls();
            display.lcd.locate(0, 0);
            display.lcd.printf(hm10.currentReadBuffer);
            // decodingProcedure();
        }

        // float turning_speed = 10.0f;
        // float forward_speed = 30.0f;

        // if (stadia.leftJoystickY == 0.0f)
        //     turning_speed = 20.0f;
        // else turning_speed = 10.0f;

        // right_wheel.speed(stadia.leftJoystickY * forward_speed + stadia.rightJoystickX * turning_speed);
        // left_wheel.speed(stadia.leftJoystickY * forward_speed + -stadia.rightJoystickX * turning_speed);

        // right_wheel.update();
        // left_wheel.update();
    }
}