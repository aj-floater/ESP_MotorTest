#include "mbed.h" 
#include "stdio.h"

void floatToString(float value, char *buffer) {
    // Handle negative numbers
    int index = 0;
    if (value < 0) {
        buffer[index++] = '-';
        value = -value;
    }

    // Extract integer part
    int intPart = (int)value;
    float decimalPart = value - intPart;

    // Convert integer part to string (manual itoa)
    int intTemp = intPart;
    int intLen = 0;
    do {
        intLen++;
        intTemp /= 10;
    } while (intTemp > 0);

    for (int i = intLen - 1; i >= 0; i--) {
        buffer[index + i] = '0' + (intPart % 10);
        intPart /= 10;
    }
    index += intLen;

    // Add decimal point
    buffer[index++] = '.';

    int precision = 4;

    float scale = 1;
    for (int i = 0; i < precision; i++) scale *= 10;

    // Convert fractional part (2 decimal places)
    int fracPart = (int)(decimalPart * scale + 0.5);  // Round properly

    // Convert fractional part
    for (int i = precision - 1; i >= 0; i--) {
        buffer[index + i] = '0' + (fracPart % 10);
        fracPart /= 10;
    }
    index += precision;

    // Null-terminate string
    buffer[index] = '\0';
}

float RobotAngularV(Encoder& Encoder1, Encoder& Encoder2) {
    float RobotAngularSpeed = ((Encoder1.speed_linear() - Encoder2.speed_linear()) / 0.18f);
    return RobotAngularSpeed;}

void MoveForward(float distance, PwmOut &Motor1, PwmOut &Motor2, Integrator &I1, Integrator &I2){

    I1.reset();
    I2.reset();

    Motor1.write(0.0f);
    Motor2.write(0.0f);

    float d1, d2;

    do
    {
        d1 = I1.getIntegral();
        d2 = I2.getIntegral();

    } while ((d1 <= distance));

    Motor1.write(1.0f);
    Motor2.write(1.0f);  
}

void Turn(float angle, PwmOut &Motor1, PwmOut &Motor2, Integrator &I3, DigitalOut &Direction1, DigitalOut &Direction2){

    I3.reset();

    float a = 0.0f;

    if (angle < 0.0f)
    {
        Direction1.write(0);
        Direction2.write(1);

        Motor1.write(0.7f);
        Motor2.write(0.7f); 

        do
        {  
            a = I3.getIntegral();
        } while (a >= angle);
   
    }else{
            Direction2.write(0);
            Direction1.write(1);

            Motor1.write(0.7f);
            Motor2.write(0.7f); 
    
            do
            {
                a = I3.getIntegral();
            } while (a <= angle);
    }

    Direction1.write(1);
    Direction2.write(1);

    Motor1.write(1.0f);
    Motor2.write(1.0f);  
}

void FollowLine(PID &Left, PID &Right, PID &Position, PwmOut &Motor1, PwmOut &Motor2, float setspeed){

         float pwmL_speed=0.0f, pwmR_speed=0.0f, pwmL_position=0.0f, pwmR_position=0.0f, 
         total_l_speed = 0, total_r_speed = 0, total_r_speed_b = 0, total_l_speed_b = 0,
         pwmL_b=0, pwmR_b=0, a = 0.0f, speed = 0, pwmL, pwmR;


        pwmL_speed = ((Left.get_output() - 1.8295)/(-2.0257));//true
        pwmR_speed = ((Right.get_output() - 2.27)/(-2.47));//true

        speed = ((setspeed*7) * (-Position.get_output()))/(52);

        if (speed > 0.0f)
        {

            total_l_speed = (Left.get_output()) + abs(speed);
            total_r_speed_b = (Right.get_output()) - abs(speed);
            pwmL = (total_l_speed - 1.8295)/(-2.0257);
            pwmR_b = (total_r_speed_b - 2.27)/(-2.47);

            Motor2.write(pwmL);
            Motor1.write(pwmR_b);
            
        }
        if (speed < 0.0f)
        {
            
            total_r_speed = (Right.get_output()) + abs(speed);
            total_l_speed_b = (Left.get_output()) - abs(speed);
            pwmR = (total_r_speed - 2.27)/(-2.47);
            pwmL_b = (total_l_speed_b - 1.8295)/(-2.0257);

            Motor1.write(pwmR);
            Motor2.write(pwmL_b);
           
        }
        if (speed == 0)
        {
            Motor2.write(pwmL_speed);
            Motor1.write(pwmR_speed);
        }
    }