#include "mbed.h" 

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

    Motor1.write(0.7f);
    Motor2.write(0.7f);

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

    if (angle < 0.0f)
    {
        Direction1.write(0);

        Motor1.write(0.7f);
        Motor2.write(0.7f); 

        float a;

        do
        {
            a = I3.getIntegral();
            
        } while (I3.getIntegral() >= angle);
   
    }else{
            Direction2.write(0);

            Motor1.write(0.7f);
            Motor2.write(0.7f); 

            float a;
    
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