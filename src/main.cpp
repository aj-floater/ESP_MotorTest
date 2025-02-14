#include "mbed.h"
#include "C12832.h"
#include "classes.h"
#include "functions.h"



int main(void){


    // Pointers to functions
        float (Encoder::*PtrToEncoderLSpeed)() = &Encoder::speed_linear;
        float (Encoder::*PtrToEncoderASpeed)() = &Encoder::speed_angular;

    
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
    Encoder Encoder1(PA_11,PA_12);
    //Encoder Encoder2(,);

    // Integrator setup
        Integrator I1(10.0f);
        I1.start(&Encoder1, PtrToEncoderLSpeed);
        Integrator I2(10.0f);
        I1.start(&Encoder2, PtrToEncoderLSpeed);

    // float issue
        char buffer[50];

    while(1){

        do
        {
            Motor1.write(0.3f);
            Motor2.write(0.3f);

        } while ((I1.getvalue() <= 0.5)&&(I2.getvalue() <= 0.5));

        I1.stop();
        I2.stop();

        Motor1.write(1.0f);
        Motor2.write(1.0f);
        
        while(1){}

        
     
    }
}