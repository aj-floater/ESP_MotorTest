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
        Encoder Encoder1(PC_2,PC_3,0);   //right
        Encoder Encoder2(PC_7,PA_9,1);   //left

        Encoder1.initialise();
        Encoder2.initialise();

        // Integrator setup
        // Integrator I1(1000.0f);
        // I1.start(callback(&Encoder1,&Encoder::speed_linear));
        // Integrator I2(1000.0f);
        // I2.start(callback(&Encoder2,&Encoder::speed_linear));

        // Integrator I3(1000.0f);
        // auto RobotAngularV_Callback = [&]() -> float {
        //     return RobotAngularV(Encoder1, Encoder2);
        // };
        // I3.start(RobotAngularV_Callback);

        // float issue
        char buffer1[50];
        char buffer2[50];

        float speed, distance;

        // PID
        PID Right(1.0f, 1.0f, 0.0f, 0.0f, 4.0f, 0.0f, 100); //float setpoint,float Kp,float Ki,float Kd,float max_out,float min_out,float freq
        PID Left(1.0f, 1.0f, 0.0f, 0.0f, 4.0f, 0.0f, 100);

        Left.start(callback(&Encoder2,&Encoder::speed_linear));
        Right.start(callback(&Encoder1,&Encoder::speed_linear));

        float pwmL, pwmR;


    while(1){

         pwmL = ((Left.get_output() - 4.2)/(-4.28));
         pwmR = ((Right.get_output() - 4.2)/(-4.28));

        Motor1.write(pwmR);
        Motor2.write(pwmL);

        floatToString(Left.get_output(), buffer1);
        floatToString(Right.get_output(), buffer2);



        printf("Left PID out: %s,   Right PID out %s \n", buffer1, buffer2);
    }
}
