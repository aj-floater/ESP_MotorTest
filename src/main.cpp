#include "mbed.h"
#include "C12832.h"

int main(void){
    // ----------------------------
    DigitalOut Bipolar1(PB_13);
    Bipolar1.write(0);

    DigitalOut Direction1(PB_14);
    Direction1.write(1);
    
    PwmOut Motor1(PC_6);

    DigitalOut Bipolar2(PB_15);
    Bipolar2.write(0);

    DigitalOut Direction2(PB_1);
    Direction2.write(1);

    PwmOut Motor2(PC_8);

    DigitalOut Enable(PB_2);
    Enable.write(1);
    // ----------------------------

    Motor1.period_us(45);
    Motor2.period_us(45);

    Motor1.write(0.6f);
    Motor2.write(0.6f);

    while(1){

        // printf("The speed of wheel 1 is: %f \n", Encoder1.speed());

    }
}