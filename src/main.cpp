#include "mbed.h"

#include "potentiometer.h"
#include "input.h"
#include "encoder.h"
#include "wheel.h"
#include "display.h"

Integrator i_left(1000.0f);
Integrator i_right(1000.0f);
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

void MoveForward(float speed, float distance) {

    left_wheel.speed(0.0f);
    right_wheel.speed(0.0f);

    update();
    
    i_left.reset();
    i_right.reset();
    
    Kp_straight = 2.35f;  // Tune this parameter

    do {
        // 1. Calculate wheel travel difference
        float error = i_left.getIntegral() - i_right.getIntegral();
        float adjustment = Kp_straight * error;

        // 2. Apply correction
        float left_speed = speed - adjustment;
        float right_speed = speed + adjustment;

        // 3. Ensure speeds stay within valid range
        if (left_speed > speed) left_speed = speed;
        if (right_speed > speed) right_speed = speed;
        if (left_speed < -speed) left_speed = -speed;
        if (right_speed < -speed) right_speed = -speed;

        // 4. Set adjusted speeds
        left_wheel.speed(left_speed);
        right_wheel.speed(right_speed);

        update();

    } while ((i_left.getIntegral() <= distance) && (i_right.getIntegral() <= distance));

    // Stop the wheels
    left_wheel.speed(0.0f);
    right_wheel.speed(0.0f);
    update();

    i_left.reset();
    i_right.reset();
}

void Turn(float speed, float angle){
    a = angle;
    
    left_wheel.speed(0.0f);
    right_wheel.speed(0.0f);
    i_angular.reset();
    
    update();
    
    // Define a deadband threshold (e.g., 0.05 radians)
    // deadband = 0.05f;

    while (fabsf(i_angular.getIntegral() - angle) > 0.1f) {
        // Compute error
        float angleError = angle - i_angular.getIntegral();
        float diff = fabsf(angleError);
        
        // If the error is extremely small, break out to avoid oscillations.
        // if (fabsf(angleError) < deadband) {
        //     break;
        // }

        // Proportional control: compute a rampdown speed
        // Kp = 11.0f;
        float targetSpeed = Kp * diff;
        if (targetSpeed > speed)
            targetSpeed = speed;

        // Define the 30° threshold in radians
        // float degThreshold = 50.0f * (M_PI / 180.0f);  // ≈ 0.523599 radians
        // if (diff < degThreshold && targetSpeed > speed * 0.8f) {
        //     targetSpeed = speed * (0.5f+(diff/degThreshold)*0.3f);
        // }

        // Ensure we don't drop below a minimum speed factor
        // minScaleFactor = 0.8f;
        if (targetSpeed < speed * minScaleFactor)
            targetSpeed = speed * minScaleFactor;

        // Set wheel speeds based on error sign
        if (angleError > 0.0f) {
            left_wheel.speed(-targetSpeed);
            right_wheel.speed(targetSpeed);
        } else {
            left_wheel.speed(targetSpeed);
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

    LeftPot.setRange(0.0f, 15.0f); // 0.005
    RightPot.setRange(0.0f, 1.0f); // 0.0014

    i_left.start(callback(&left_wheel,&Wheel::measured_speed_linear));
    i_right.start(callback(&right_wheel,&Wheel::measured_speed_linear));

    auto RobotAngularV_Callback = [&]() -> float {
        return RobotAngularV(right_wheel.encoder, left_wheel.encoder);
    };
    i_angular.start(RobotAngularV_Callback);

    update();

    float speed = 30.0f;
    float turn_speed = 12.0f;
    float wait_time = 0.00f; // in seconds
    wait_time = wait_time * 1'000'000;
    float distance = 1.0f;

    while(1){
        wait_us(wait_time);
        MoveForward(speed, distance);
        wait_us(wait_time);
        Turn(turn_speed, 3.1415f/2.0f);

        wait_us(wait_time);
        MoveForward(speed, distance);
        wait_us(wait_time);
        Turn(turn_speed, 3.1415f/2.0f);

        wait_us(wait_time);
        MoveForward(speed, distance);
        wait_us(wait_time);
        Turn(turn_speed, 3.1415f/2.0f);
        
        wait_us(wait_time);
        MoveForward(speed, distance);

        wait_us(wait_time);
        Turn(turn_speed, -3.1415f);

        wait_us(wait_time);
        MoveForward(speed, distance);
        wait_us(wait_time);
        Turn(turn_speed, -3.1415f/2.0f);

        wait_us(wait_time);
        MoveForward(speed, distance);
        wait_us(wait_time);
        Turn(turn_speed, -3.1415f/2.0f);
        
        wait_us(wait_time);
        MoveForward(speed, distance);
        wait_us(wait_time);
        Turn(turn_speed, -3.1415f/2.0f);
        
        wait_us(wait_time);
        MoveForward(speed, distance);
        wait_us(wait_time);
        Turn(turn_speed, 3.1415f);

        while (true) { /* FINISHED */ }
    }
}