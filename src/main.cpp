#include "mbed.h"

#include "potentiometer.h"
#include "input.h"
#include "encoder.h"
#include "wheel.h"
#include "display.h"

Integrator i_left(100.0f);
Integrator i_right(100.0f);
Integrator i_angular(100.0f);

void update(){
    right_wheel.update();
    left_wheel.update();
    
    LeftPot.update();
    RightPot.update();
    modify(LeftPot.getCurrentSampleNorm(), RightPot.getCurrentSampleNorm());

    display.refresh();

    char buffer[50];
    // floatToString(right_wheel.desired_speed, buffer);
    // printf(">rds:%s,", buffer);
    floatToString(right_wheel.measured_speed_linear(), buffer);
    printf(">rs:%s,", buffer);
    // floatToString(right_wheel.control_output, buffer);
    // printf("rco:%s\n", buffer);

    // floatToString(left_wheel.desired_speed, buffer);
    // printf(">lds:%s,", buffer);
    floatToString(left_wheel.measured_speed_linear(), buffer);
    printf("ls:%s\n", buffer);
    // floatToString(left_wheel.control_output, buffer);
    // printf("lco:%s\n", buffer);

    // char buffer[50];
    floatToString(i, buffer);
    printf(">i:%s,", buffer);
    floatToString(a, buffer);
    printf("a:%s\n", buffer);
}

void MoveForward(float speed, float distance){

    left_wheel.speed(0.0f);
    right_wheel.speed(0.0f);

    update();
    
    i_left.reset();
    i_right.reset();
    
    do
    {
        float integralCompensation1 = 0;//(i_right.getIntegral() - i_left.getIntegral()) * - 20.5f;
        float integralCompensation2 = 0;//(i_left.getIntegral() - i_right.getIntegral()) * 100.0f;
        
        left_wheel.speed(speed + integralCompensation1);
        right_wheel.speed(speed + integralCompensation2);

        update();

    } while ((i_left.getIntegral() <= distance) && (i_right.getIntegral() <= distance));
    
    left_wheel.speed(0.0f);
    right_wheel.speed(0.0f);

    update();

    i_left.reset();
    i_right.reset();
}

float getAngularVelocity(float right_speed, float left_speed){
    float speed_difference = right_speed - left_speed;
    float angular_speed = speed_difference / 0.173f;

    return angular_speed;
}

void Turn(float speed, float angle){
    a = angle;
    
    left_wheel.speed(0.0f);
    right_wheel.speed(0.0f);
    i_angular.reset();
    
    update();
    
    while (fabsf(i_angular.getIntegral() - angle) > 0.1f) {

        // 1. Compute error
        float angleError = angle - i_angular.getIntegral();
        float diff       = fabsf(angleError);
    
        // 2. Proportional control: compute a rampdown speed
        // float Kp = 11.0f;             // Proportional gain - tune as needed
        float targetSpeed = Kp * diff;
        if (targetSpeed > speed)
            targetSpeed = speed;
    
        // Ensure we don't drop below a minimum speed factor
        // float minScaleFactor = 0.7f; // At least 80% of speed, tune as needed
        if (targetSpeed < speed * minScaleFactor)
            targetSpeed = speed * minScaleFactor;
    
        // 3. Set the speeds based on the sign of the error
        if (angleError > 0.0f) {
            left_wheel.speed(- 0.9f * targetSpeed);
            right_wheel.speed( targetSpeed);
        } else {
            left_wheel.speed( 0.9f * targetSpeed);
            right_wheel.speed(-targetSpeed);
        }
    
        update();
    }
    
    
    left_wheel.speed(0.0f);
    right_wheel.speed(0.0f);
    i_angular.reset();

    update();
}

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

    right_wheel.speed(0.0f);
    left_wheel.speed(0.0f);

    LeftPot.setRange(7.0f, 20.0f); // 0.005
    RightPot.setRange(0.4f, 0.9f); // 0.0014

    // modify(10.0f, 10.0f);

    i_left.start(callback(&left_wheel,&Wheel::measured_speed_linear));
    i_right.start(callback(&right_wheel,&Wheel::measured_speed_linear));

    auto RobotAngularV_Callback = [&]() -> float {
        return RobotAngularV(right_wheel.encoder, left_wheel.encoder);
    };
    i_angular.start(RobotAngularV_Callback);

    update();

    while(1){
        wait_us(500000);
        MoveForward(12.0f, 0.5f);
        wait_us(500000);
        Turn(12.0f, 3.1415f/2.0f);

        // update();
    }
}