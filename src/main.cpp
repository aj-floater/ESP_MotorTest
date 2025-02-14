#include "mbed.h"
#include "C12832.h"
#include "classes.h"
#include "functions.h"


Encoder Encoder1(PA_11,PA_12);
//Encoder Encoder2(,);


volatile float summation = 0;
void integration_updateISR_r(void){
summation = summation + 0.001f * (Encoder1.speed_linear());
}

float returnone(void){
    return 1.0f;
}


int main(void){

    // Pointers to functions
        //float (Encoder::*EncoderSpeedLinearPTR)();
        float (* returnonePTR)();
        returnonePTR = returnone;

    
    // Pin Setup
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
    

    // Encoders setup

    // Integrator setup
        Integrator I1(10.0f);
        I1.start(returnonePTR);

    // float issue
        char buffer[50];

    while(1){

        floatToString(I1.getvalue(),buffer);

        printf("Integration = %s \n", buffer);


        wait_us(100000);
        
     
    }
}