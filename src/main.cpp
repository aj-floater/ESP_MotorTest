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
    Encoder Encoder1(PA_11,PA_12,0);
    Encoder Encoder2(PC_7,PA_9,1);

    Encoder1.initialise();
    Encoder2.initialise();

    // Integrator setup
        Integrator I1(100.0f);
        I1.start(callback(&Encoder1,&Encoder::speed_linear));
        Integrator I2(10.0f);
        I2.start(callback(&Encoder2,&Encoder::speed_linear));

        Integrator I3(10.0f);
        auto RobotAngularV_Callback = [&]() -> float {
            return RobotAngularV(Encoder1, Encoder2);
        };
        I3.start(RobotAngularV_Callback);

    // float issue
        char buffer1[50];
        char buffer2[50];

        float speed, distance;

        


    while(1){

        // Motor1.write(1.0f);
        // Motor2.write(1.0f);


        // wait_us(5000000);



        // do
        // {
        //     Motor1.write(0.7f);
        //     Motor2.write(0.7f);

        // } while ((I1.getIntegral() <= 0.5)&&(I2.getIntegral() <= 0.5));

        // I1.stop();
        // I2.stop();

        // Motor1.write(1.0f);
        // Motor2.write(1.0f);

        // wait_us(1000000);


        // I3.reset();
        // Direction2.write(0);
        // do{
        // Motor1.write(0.7f);
        // Motor2.write(0.7f);
        // }while(I3.getIntegral() <= 3.141519f/2.0f);

        // Direction2.write(1);
        // Motor1.write(1.0f);
        // Motor2.write(1.0f);

        // do
        // {
        //     Motor1.write(0.7f);
        //     Motor2.write(0.7f);

        // } while ((I1.getIntegral() <= 0.5)&&(I2.getIntegral() <= 0.5));

        
        // while(1){}       
        
        floatToString(RobotAngularV(Encoder1, Encoder2), buffer1);
        floatToString(I3.getIntegral(), buffer2);
        printf("AngularV: %s,  Angle: %s,  R_direction: %d,  L_direction: %d\n", buffer1, buffer2, Encoder1.direction, Encoder2.direction);

        wait_us(100000);
     
    }
}