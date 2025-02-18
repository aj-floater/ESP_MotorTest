#include "mbed.h"

#include "potentiometer.h"
#include "input.h"
#include "encoder.h"
#include "wheel.h"
#include "display.h"

int main(void){
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

    // right_wheel.speed(30.0f);
    // left_wheel.speed(30.0f);

    LeftPot.setRange(-50.0f, 50.0f); // 0.005
    RightPot.setRange(-50.0f, 50.0f); // 0.0014

    modify(5.0f, 5.0f);

    while(1){
        right_wheel.update();
        left_wheel.update();
        
        LeftPot.update();
        RightPot.update();
        // modify(LeftPot.getCurrentSampleNorm(), RightPot.getCurrentSampleNorm());

        display.refresh();

        char buffer[50];
        floatToString(right_wheel.desired_speed, buffer);
        printf(">ds:%s,", buffer);

        floatToString(right_wheel.measured_speed_angular(), buffer);
        printf("rs:%s,", buffer);
        floatToString(right_wheel.control_output, buffer);
        printf("co:%s\n", buffer);
    }
}