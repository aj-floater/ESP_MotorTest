#include "mbed.h"
#include "C12832.h"
#include "classes.h"


Encoder Encoder1(PA_11,PA_12);
//Encoder Encoder2(,);


volatile float summation = 0;
void integration_updateISR_r(void){
summation = summation + 0.001f * (Encoder1.speed_linear());
}




int main(void){
    // Pin Setup
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

    // Encoders setup



    //Timers
    Ticker Integrator_t_r;
    Integrator_t_r.attach(&integration_updateISR_r,0.01f);





    while(1){
        
     
    }
}