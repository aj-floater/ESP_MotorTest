#include "mbed.h"
#include "C12832.h"
class Encoder {

    public:

    InterruptIn ChanelA, ChanelB;
    Ticker Encoder_dt; 
    float dt = 0.3;
    volatile float EncoderTick;
    volatile int countA = 0, countB = 0;
    

    Encoder(PinName ChA, PinName ChB) : ChanelA(ChA), ChanelB(ChB){}

    void initialise(void){
        
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

// This section will define a Wheel class, allowing users to set a target angular (or linear) velocity.
// The PID controller will continuously calculate the error between the desired and actual speed, 
// adjusting the PWM duty cycle accordingly to maintain the set speed.
//
// The PID control loop will:
// 1. Measure the current speed from sensors (e.g., encoders).
// 2. Compute the error by comparing the measured speed to the target speed.
// 3. Apply PID corrections to adjust motor power via PWM.
class Wheel {
private:
public:
    float desired_speed;
    float measured_speed;

    float proportional_gain;
    float integral_gain;
    float derivative_gain;

    float control_output;

    Timer timer;
    float previous_time;
    float previous_error;
    float previous_error_2;
    float previous_integral;

    Encoder encoder;
    PwmOut motor;

    Wheel(float Kp, float Ki, float Kd, PinName ChA, PinName ChB, PinName pwm, float frequency) : 
        proportional_gain(Kp), 
        integral_gain(Ki), 
        derivative_gain(Kd), 
        encoder(ChA, ChB),
        motor(pwm)
    {
        encoder.initialise();
        timer.start();

        motor.period_us(45);
        motor.write(0.8f);

        desired_speed = 0.0f;
        measured_speed = 0.0f;
        control_output = 1.0f;
    }

    float measured_speed_angular() { return encoder.speed_angular(); };
    float measured_speed_linear() { return encoder.speed_linear(); };

    float deltaTime(){
        int dt = timer.read_us() - previous_time;
        timer.reset();
        return dt / 1'000'000.0f;  // Convert µs to seconds
    }

    // Set speed in rad/s
    float speed(float s){
        desired_speed = s;
        return 0;
    }
    float speed(){
        return desired_speed;
    }

    // Calculate the error e(t) = Desired Speed − Measured Speed
    float error(){
        return measured_speed_angular() - desired_speed;
    }

    // Calculate the control output (ie the PWM duty cycle)
    void piControl(){
        float Et = error(); // gets the error at the current time
        float dt = deltaTime();

        float proportional_term = proportional_gain * Et;
        float integral_term = integral_gain * dt * Et - previous_error * proportional_gain;

        control_output += proportional_term + integral_term;

        // Setup values for next iteration
        previous_error = Et;
    }

    void update(){
        // Clamp final output to 0.0f and 1.0f
        piControl();

        if (control_output < 0.0f) control_output = 0.0f;
        else if (control_output > 1.0f) control_output = 0.8f;

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

Wheel left_wheel(0.0192f, 0.0104f, 0.0f/*0.00312f*/, PA_12, PA_11, PC_6, 1.0f);

bool cls = false;
C12832 lcd(D11, D13, D12, D7, D10);
char buffer[50];
void refreshDisplay() {
    if (cls) {
        lcd.cls();
        cls = false;
    }

    lcd.locate(0,0);
    // lcd.printf("Hello World");
    floatToString(left_wheel.proportional_gain, buffer);
    lcd.printf("Kp: %s\n", buffer);
    lcd.locate(80,0);
    floatToString(left_wheel.control_output, buffer);
    lcd.printf("o: %s\n", buffer);

    lcd.locate(0,10);
    floatToString(left_wheel.speed(), buffer);
    lcd.printf("s: %s\n", buffer);
    lcd.locate(80,10);
    floatToString(left_wheel.measured_speed_angular(), buffer);
    lcd.printf("ms: %s\n", buffer);

    lcd.locate(0,20);
    floatToString(left_wheel.error(), buffer);
    lcd.printf("e: %s\n", buffer);
}

// kp debugging
float dKp = 0.0001f;
void leftISR(){
    left_wheel.proportional_gain = left_wheel.proportional_gain - dKp;
    cls = true;
}
void rightISR(){
    left_wheel.proportional_gain = left_wheel.proportional_gain + dKp;
    cls = true;
}
void upISR(){
    // left_wheel.control_output += 0.1f;
    left_wheel.speed(left_wheel.speed()+1);
    cls = true;
}
void downISR(){
    // left_wheel.control_output -= 0.1f;
    left_wheel.speed(left_wheel.speed()-1);
    cls = true;
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

    PwmOut Motor2(PC_8);

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
    left.rise(&leftISR);
    right.rise(&rightISR);

    // Motor1.write(1.0f);

    Motor2.period_us(45);
    Motor2.write(1.0f);

    left_wheel.speed(30.0f);

    while(1){
        left_wheel.update();

        refreshDisplay();
    }
}