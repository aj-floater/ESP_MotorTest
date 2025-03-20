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
        PID Right(0.3f, 1.8f, 4.9f, 5.8f, 4.0f, -4.0f, 3300); //float setpoint,float Kp,float Ki,float Kd,float max_out,float min_out,float freq
        PID Left(0.3f, 1.8f, 4.9f, 5.8f, 4.0f, -4.0f, 3300); // p=1.8, i=4.5, d=0

        Left.start(callback(&Encoder2,&Encoder::speed_linear));
        Right.start(callback(&Encoder1,&Encoder::speed_linear));

        float pwmL, pwmR;


        char linear1_buffer[20];
        char linear2_buffer[20];

        Timer t;



    while(1){

        pwmL = ((Left.get_output() - 1.8295)/(-2.0257));//true
        pwmR = ((Right.get_output() - 2.27)/(-2.47));//true

        while(1){
            Motor2.write(1.0f);
            Motor1.write(0.8f);

            float linear1 =  Encoder1.speed_linear();
            float linear2 =  Encoder2.speed_linear();

            floatToString(linear1, linear1_buffer);
            printf(">Encoder1:%s\n", linear1_buffer);
            floatToString(linear2, linear2_buffer);
            printf(">Encoder2:%s\n", linear2_buffer);

        }

         float time = 0;
         t.start();

         while (time <= 5000)
         {
            time = t.read_ms();

            Left.setpoint = 0.3;

            pwmL = ((Left.get_output() - 1.8295)/(-2.0257));//true

            Motor2.write(pwmL);

            float linear1 =  Left.setpoint;//Encoder1.speed_linear();
            float linear2 =  Encoder2.speed_linear();
            floatToString(linear1, linear1_buffer);
            printf(">Encoder1:%s\n", linear1_buffer);
            floatToString(linear2, linear2_buffer);
            printf(">Encoder2:%s\n", linear2_buffer);
         }

         while (time <= 10000)
         {
            time = t.read_ms();

            Left.setpoint = 2.6;

            pwmL = ((Left.get_output() - 1.8295)/(-2.0257));//true

            Motor2.write(pwmL);

            float linear1 =  Left.setpoint;//Encoder1.speed_linear();
            float linear2 =  Encoder2.speed_linear();
            floatToString(linear1, linear1_buffer);
            printf(">Encoder1:%s\n", linear1_buffer);
            floatToString(linear2, linear2_buffer);
            printf(">Encoder2:%s\n", linear2_buffer);
         }

         while (time <= 15000)
         {
            time = t.read_ms();

            Left.setpoint = 1.0f;

            pwmL = ((Left.get_output() - 1.8295)/(-2.0257));//true

            Motor2.write(pwmL);

            float linear1 =  Left.setpoint;//Encoder1.speed_linear();
            float linear2 =  Encoder2.speed_linear();
            floatToString(linear1, linear1_buffer);
            printf(">Encoder1:%s\n", linear1_buffer);
            floatToString(linear2, linear2_buffer);
            printf(">Encoder2:%s\n", linear2_buffer);
         }

         t.reset();
         



        // floatToString(Left.get_output(), buffer1);
        // floatToString(Encoder2.speed_linear(), buffer2);


        // printf(">Lpidout: %s,Lspeed: %s\r\n", buffer1, buffer2);


        float linear1 =  1.0f;//Encoder1.speed_linear();
        float linear2 =  Encoder2.speed_linear();
        floatToString(linear1, linear1_buffer);
        printf(">Encoder1:%s\n", linear1_buffer);
        floatToString(linear2, linear2_buffer);
        printf(">Encoder2:%s\n", linear2_buffer);
    }
}
