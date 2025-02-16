#include "mbed.h"
#include "C12832.h"
#include "stdio.h"

// This class will create objects for the encoders. The constructor expects the pins of ChA and ChB respectively.
// object.speed_linear() will return the linear speed of the encoder in m/s.
// object.speed_angular() will return the angular speed of the encoder in rad/s.
class Encoder {

    public:

    InterruptIn ChanelA, ChanelB;
    Ticker Encoder_dt; 
    float dt = 0.5;
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


//This code is invalid yet.
class Integrator {

    private:
    float frequency, signal;
    Ticker I_Cycle;

    public:

    Integrator(float f):frequency(f){}

    void start(float *signal){
        I_Cycle.attach(callback(this,&Integrator::IcycleISR),(1.0f/frequency));
    }


    protected:
    void IcycleISR(void){}
   
};

void floatToString(float value, char *buffer) {
    int int_part = (int)value;  // Extract integer part
    float decimal_part = value - int_part;  // Get fractional part

    if (decimal_part < 0) decimal_part = -decimal_part;  // Handle negatives
    int decimal_int = (int)(decimal_part * pow(10, 3) + 0.5); // Scale & round

    // Format as string
    sprintf(buffer, "%d.%.3d", int_part, decimal_int);
}

// Encoder encoder1(PA_12, PA_11);
Wheel right_wheel(0.0192f, 0.0f, 0.0192f, PA_12, PA_11, PC_6);
// Encoder encoder2(PC_7, PA_9);
Wheel left_wheel(0.0192f, 0.0f, 0.0192f, PC_7, PA_9, PB_1);

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
    floatToString(right_wheel.proportional_gain, buffer);
    lcd.printf("Kp: %s\n", buffer);
    lcd.locate(80,0);
    floatToString(right_wheel.control_output, buffer);
    lcd.printf("o: %s\n", buffer);

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
float Kp = 0.0f;
float dKp = 0.1f;
void leftISR(){
    right_wheel.proportional_gain = right_wheel.proportional_gain - dKp;
    cls = true;
}
void rightISR(){
    right_wheel.proportional_gain = right_wheel.proportional_gain + dKp;
    cls = true;
}
void upISR(){
    // right_wheel.control_output += 0.1f;
    right_wheel.speed(right_wheel.speed()+1);
    cls = true;
}
void downISR(){
    // right_wheel.control_output -= 0.1f;
    right_wheel.speed(right_wheel.speed()-1);
    cls = true;
}

int main(void){

    printf("Program started!\n");

    // Pin Setup
    // ----------------------------
    DigitalOut Bipolar1(PB_13);
    Bipolar1.write(0);

    DigitalOut Direction1(PB_14);
    Direction1.write(1);
    
    // PwmOut Motor1(PC_6);

    DigitalOut Bipolar2(PB_15);
    Bipolar2.write(0);

    DigitalOut Direction2(PC_8);
    Direction2.write(1);

    // PwmOut Motor2(PC_8);

    DigitalOut Enable(PB_2);
    Enable.write(1);
    // ----------------------------

    // Joystick
    InterruptIn centre(D4);
    InterruptIn up(PA_4);
    InterruptIn down(PB_0);
    InterruptIn left(PC_1);
    InterruptIn right(D4);

    up.rise(&upISR);
    down.rise(&downISR);
    left.rise(&leftISR);
    right.rise(&rightISR);

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