#include "mbed.h"
#include "C12832.h"

bool screenNeedsRefresh = false;

// Potentiometer class
class Potentiometer {
private:
    AnalogIn inputSignal;          // Analog input pin
    float VDD;                     // Reference voltage
    float currentSampleNorm;       // Normalized sample (0.0 to 1.0)
    float currentSampleVolts;      // Sample in volts

    // Function to truncate a float to 2 decimal places
    float truncateTo2DP(float value) const {
        return floor(value * 100) / 100.0f;
    }

public:
    Potentiometer(PinName pin, float v) : inputSignal(pin), VDD(v) {
        currentSampleNorm = inputSignal.read();
    }

    void sample(void) {
        if (inputSignal.read() == currentSampleNorm){
            screenNeedsRefresh = true;
        }

        currentSampleNorm = inputSignal.read();  // Read normalized value
        currentSampleVolts = currentSampleNorm * VDD; // Convert to volts
    }

    float getCurrentSampleNorm(void) {
        return truncateTo2DP(currentSampleNorm);
    }

    float getCurrentSampleVolts(void) {
        return currentSampleVolts;
    }
};

// SamplingPotentiometer class inheriting from Potentiometer
class SamplingPotentiometer : public Potentiometer {
private:
    float samplingFrequency;        // Sampling frequency in Hz
    float samplingPeriod;           // Sampling period in seconds
    Ticker sampler;                 // Ticker object for periodic sampling

public:
    SamplingPotentiometer(PinName p, float v, float fs)
        : Potentiometer(p, v), samplingFrequency(fs) {
        samplingPeriod = 1.0f / samplingFrequency;  // Calculate period
        sampler.attach(callback(this, &SamplingPotentiometer::sample), samplingPeriod);
    }
};

class Encoder {
    public:

    InterruptIn ChanelA, ChanelB;
    Ticker Encoder_dt; 
    float dt = 0.1;
    volatile float EncoderTick;
    volatile int countA = 0, countB = 0;
    

    Encoder(PinName ChA, PinName ChB) : 
        ChanelA(ChA), 
        ChanelB(ChB)
    {
        Encoder_dt.attach(callback(this,&Encoder::EncoderCycleISR), dt);
        
        ChanelA.rise(callback(this,&Encoder::ChanelA_countISR));
        ChanelB.rise(callback(this,&Encoder::ChanelB_countISR));
    }

    float speed_linear(void){
        float radPERseconds = (EncoderTick / 256.0f) * 2.0f * 3.141519f;
        float wheelVelocity = 0.08f * radPERseconds;
        return wheelVelocity;
    }
    float speed_angular(void){
        float radPERseconds = (EncoderTick / 256.0f) * 2.0f * 3.141519f;
        return radPERseconds;
    }

    protected:
    void EncoderCycleISR(void){
        EncoderTick = (float(countA)/dt);
        countA = 0; countB = 0;
    }

    void ChanelA_countISR(void){countA++;}
    void ChanelB_countISR(void){countB++;}

};
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

void modify(float Kp, float Kd){
    right_wheel.proportional_gain = Kp;
    left_wheel.proportional_gain = Kp;
    
    right_wheel.derivative_gain = Kd;
    left_wheel.derivative_gain = Kd;
}

float Kp = 0.0192f;
float Kd = 0.0001f;

Wheel right_wheel(0.0192f, 0.0f, 0.0192f, PA_12, PA_11, PC_6);
Wheel left_wheel(0.0192f, 0.0f, 0.0192f, PC_7, PA_9, PC_8);

C12832 lcd(D11, D13, D12, D7, D10);
char buffer[50];
void refreshDisplay() {
    if (screenNeedsRefresh) {
        lcd.cls();
        screenNeedsRefresh = false;
    }

    lcd.locate(0,0);
    // lcd.printf("Hello World");
    floatToString(right_wheel.proportional_gain, buffer);
    lcd.printf("Kp: %s\n", buffer);
    lcd.locate(80,0);
    floatToString(right_wheel.control_output, buffer);
    lcd.printf("Kd: %s\n", buffer);

    lcd.locate(0,10);
    floatToString(right_wheel.speed(), buffer);
    lcd.printf("s: %s\n", buffer);
    lcd.locate(80,10);
    floatToString(right_wheel.measured_speed_angular(), buffer);
    lcd.printf("ms: %s\n", buffer);

    lcd.locate(0,20);
    floatToString(right_wheel.error(), buffer);
    lcd.printf("e: %s\n", buffer);
}

// kp debugging
void upISR(){
    // right_wheel.control_output += 0.1f;
    right_wheel.speed(right_wheel.speed()+1);
    screenNeedsRefresh = true;
}
void downISR(){
    // right_wheel.control_output -= 0.1f;
    right_wheel.speed(right_wheel.speed()-1);
    screenNeedsRefresh = true;
}

int main(void){
    // Pin Setup
    // ----------------------------
    DigitalOut Bipolar1(PB_13);
    Bipolar1.write(0);

    DigitalOut Direction1(PB_14);
    Direction1.write(1);
    
    // PwmOut Motor1(PC_6);

    DigitalOut Bipolar2(PB_15);
    Bipolar2.write(0);

    DigitalOut Direction2(PB_1);
    Direction2.write(1);

    // PwmOut Motor2(PC_8);

    DigitalOut Enable(PB_2);
    Enable.write(1);
    // ----------------------------

    // Joystick
    // InterruptIn centre(D4);
    InterruptIn up(PA_4);
    InterruptIn down(PB_0);
    InterruptIn left(PC_1);
    InterruptIn right(D4);

    up.rise(&upISR);
    down.rise(&downISR);

    // Motor1.write(1.0f);
    // Motor2.write(1.0f);

    right_wheel.speed(30.0f);
    left_wheel.speed(30.0f);

    while(1){
        right_wheel.update();

        refreshDisplay();

        floatToString(right_wheel.desired_speed, buffer);
        printf(">ds:%s,", buffer);
        floatToString(right_wheel.measured_speed_angular(), buffer);
        printf("rs:%s\n", buffer);
    }
}