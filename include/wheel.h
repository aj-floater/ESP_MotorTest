#ifndef WHEEL_H  // If WHEEL_H is not defined
#define WHEEL_H  // Define WHEEL_H

#include "mbed.h"
#include "encoder.h"

// Header file content goes here
float Kp = 0.0192f;
float Kd = 0.0001f;

class Wheel {
private:
public:
    float desired_speed;
    float measured_speed;

    float proportional_gain;
    float integral_gain;
    float derivative_gain;
    
    float previous_error;
    float integral_error;

    float control_output;

    Encoder encoder;
    PwmOut motor;

    Wheel(float Kp, float Ki, float Kd, PinName ChA, PinName ChB, PinName pwm) : 
        proportional_gain(Kp), 
        integral_gain(Ki),
        derivative_gain(Kd),
        encoder(ChA, ChB),
        motor(pwm)
    {
        motor.period_us(45);
        motor.write(1.0f);

        desired_speed = 0.0f;
        measured_speed = 0.0f;
        control_output = 1.0f;
        
        previous_error = 0.0f;
        integral_error = 0.0f;
    }

    float measured_speed_angular() { 
        return encoder.speed_angular(); 
    };
    float measured_speed_linear() { 
        return encoder.speed_linear(); 
    };

    // Set speed in rad/s
    float speed(float s){
        desired_speed = s;
        return 0;
    }
    float speed(){
        return desired_speed;
    }

    // Calculate the error e(t) = Desired Speed − Measured Speed
    float error(){
        return desired_speed - measured_speed_angular();
    }

    // PID Control
    void pidControl(){
        float Et = error(); // Current error
        
        integral_error += Et; // Integrate the error
        float derivative_term = Et - previous_error; // Derivative term
        
        float proportional_term = proportional_gain * Et;
        float integral_term = integral_gain * integral_error;
        float derivative_control = derivative_gain * derivative_term;
        
        control_output = proportional_term + integral_term + derivative_control;

        // Setup values for next iteration
        previous_error = Et;
    }

    void update(){
        // Update control
        pidControl();
        // control_output = 1 - desired_speed/50;
        
        // Clamp final output to 0.0f and 1.0f
        if (control_output < 0.0f) control_output = 0.0f;
        else if (control_output > 1.0f) control_output = 1.0f;

        motor.write(control_output);
    }
};

Wheel right_wheel(0.0192f, 0.0f, 0.0192f, PA_12, PA_11, PC_6);
Wheel left_wheel(0.0192f, 0.0f, 0.0192f, PC_7, PA_9, PC_8);

#endif // End of include guard