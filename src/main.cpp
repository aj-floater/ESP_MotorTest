#include "mbed.h"
#include "C12832.h"
#include "classes.h"
#include "functions.h"


<<<<<<< Updated upstream
    // Encoders setup
    Encoder Encoder1(PA_11,PA_12, 0);
    Encoder Encoder2(PC_7,PA_9, 1);


    // Integrator setup
    Integrator I1(100.0f);
    Integrator I2(100.0f);
    Integrator I3(100.0f);


    // PWM Out
    PwmOut Motor1(PC_6);
    PwmOut Motor2(PC_8);

    //Pin Setup
    DigitalOut Bipolar1(PB_13);
    DigitalOut Direction1(PB_14);
    DigitalOut Bipolar2(PB_15);
    DigitalOut Direction2(PB_1);
    DigitalInOut Enable(PB_2);
    



void MoveForward(float distance){

    Motor1.write(1.0f);
    Motor2.write(1.0f);

    I1.reset();
    I2.reset();

    do
    {
        Motor1.write(0.7f);
        Motor2.write(0.7f);

    } while ((I1.getIntegral() <= distance) && (I2.getIntegral() <= distance));

    Motor1.write(1.0f);
    Motor2.write(1.0f);

    I1.reset();
    I2.reset();
}

void Turn(float angle){

    Motor1.write(1.0f);
    Motor2.write(1.0f);

    I3.reset();

    if(angle < 0.0f){

        Direction2.write(0);

        do{
        Motor1.write(0.7f);
        Motor2.write(0.7f);
        }while(I3.getIntegral() >= angle);

    }else{

        do{
            Motor1.write(0.7f);
            Motor2.write(0.7f);
            }while(I3.getIntegral() <= angle);
    }

    Motor1.write(1.0f);
    Motor2.write(1.0f);

    I3.reset();
}

=======



>>>>>>> Stashed changes
int main(void){

    printf("main start");


    // Pointers to functions
        float (Encoder::*PtrToEncoderLSpeed)() = &Encoder::speed_linear;
        float (Encoder::*PtrToEncoderASpeed)() = &Encoder::speed_angular;

    
    // Pin Setup
        
    Bipolar1.write(0);
    Direction1.write(1);
    Bipolar2.write(0);
    Direction2.write(1);
    Enable.write(1);
    

    

    Encoder1.initialise();
    Encoder2.initialise();

    I1.start(callback(&Encoder1,&Encoder::speed_linear));
    I2.start(callback(&Encoder2,&Encoder::speed_linear));

    auto RobotAngularV_Callback = [&]() -> float {
        return RobotAngularV(Encoder1, Encoder2);
    };
    I3.start(RobotAngularV_Callback);




    // float issue
        char buffer1[50];
        char buffer2[50];

        float speed, distance;

<<<<<<< Updated upstream
    while(1){

        Motor1.write(1.0f);
        Motor2.write(1.0f);

        wait_us(5000000);

        MoveForward(0.5f);
        Turn(3.1415/2.0f);
        MoveForward(0.5f);
        Turn(3.1415/2.0f);
        MoveForward(0.5f);
        Turn(3.1415/2.0f);
        MoveForward(0.5f);
        
        Turn(3.1415f);

        MoveForward(0.5f);
        Turn(-3.1415/2.0f);
        MoveForward(0.5f);
        Turn(-3.1415/2.0f);
        MoveForward(0.5f);
        Turn(-3.1415/2.0f);
        MoveForward(0.5f);

        Motor1.write(1.0f);
        Motor2.write(1.0f);

        while(1){}

        
=======
        
     

    while(1){

        MoveForward(0.5, Motor1, Motor2, I1, I2);
        Turn(1.57, Motor1, Motor2, I3, Direction1, Direction2); 
        MoveForward(0.5, Motor1, Motor2, I1, I2);
        Turn(1.57, Motor1, Motor2, I3, Direction1, Direction2); 
        MoveForward(0.5, Motor1, Motor2, I1, I2);
        Turn(1.57, Motor1, Motor2, I3, Direction1, Direction2); 
        MoveForward(0.5, Motor1, Motor2, I1, I2);
        
        Turn(3.1415, Motor1, Motor2, I3, Direction1, Direction2); 

        MoveForward(0.5, Motor1, Motor2, I1, I2);
        Turn(-1.57, Motor1, Motor2, I3, Direction1, Direction2); 
        MoveForward(0.5, Motor1, Motor2, I1, I2);
        Turn(-1.57, Motor1, Motor2, I3, Direction1, Direction2); 
        MoveForward(0.5, Motor1, Motor2, I1, I2);
        Turn(-1.57, Motor1, Motor2, I3, Direction1, Direction2); 
        MoveForward(0.5, Motor1, Motor2, I1, I2);

        while(1){}



      
        
        floatToString(RobotAngularV(Encoder1, Encoder2), buffer1);
        floatToString(I3.getIntegral(), buffer2);
        printf("AngularV: %s,  Angle: %s,  R_direction: %d,  L_direction: %d\n", buffer1, buffer2, Encoder1.direction, Encoder2.direction);
>>>>>>> Stashed changes

        

        

               
        
        // floatToString(RobotAngularV(Encoder1, Encoder2), buffer1);
        // floatToString(I3.getIntegral(), buffer2);
        // printf("AngularV: %s,  Angle: %s,  R_direction: %d,  L_direction: %d\n", buffer1, buffer2, Encoder1.direction, Encoder2.direction);

        // wait_us(100000);
     
    }
}