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
    float previous_error;

    float control_output;

    Encoder encoder;
    PwmOut motor;

    Wheel(float Kp, PinName ChA, PinName ChB, PinName pwm) : 
        proportional_gain(Kp), 
        encoder(ChA, ChB),
        motor(pwm)
    {
        encoder.initialise();

        motor.period_us(45);
        motor.write(1.0f);

        desired_speed = 0.0f;
        measured_speed = 0.0f;
        control_output = 1.0f;
    }

    float measured_speed_angular() { return encoder.speed_angular(); };
    float measured_speed_linear() { return encoder.speed_linear(); };

    // Set speed in rad/s
    float speed(float s){
        desired_speed = s;
        control_output = 1 - s/45;
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
    void pControl(){
        float Et = error(); // gets the error at the current time

        float proportional_term = proportional_gain * Et;
        control_output += proportional_term;

        // Setup values for next iteration
        previous_error = Et;
    }

    void update(){
        // Clamp final output to 0.0f and 1.0f
        // pControl();

        if (control_output < 0.0f) control_output = 0.0f;
        else if (control_output > 1.0f) control_output = 0.8f;

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

Wheel left_wheel(1.0f, 0.0f, 0.0f, PA_12, PA_11, PC_6);

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
float Kp = 0.0f;
float dKp = 0.1f;
void leftISR(){
    Kp = Kp - dKp;
}
void rightISR(){
    Kp = Kp + dKp;
}
void upISR(){
    // left_wheel.control_output += 0.1f;
    left_wheel.speed(left_wheel.speed()+1);
    //cls = true;
}
void downISR(){
    // left_wheel.control_output -= 0.1f;
    left_wheel.speed(left_wheel.speed()-1);
    //cls = true;
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

    DigitalOut Direction2(PB_1);
    Direction2.write(1);

    PwmOut Motor2(PC_8);

    DigitalOut Enable(PB_2);
    Enable.write(1);
    // ----------------------------

    C12832 lcd(D11, D13, D12, D7, D10);

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

    char buffer[50];
    left_wheel.speed(20.0f);

    //--------------------


    while(1){
        // Display Section

        left_wheel.update();
        refreshDisplay();

        floatToString(left_wheel.desired_speed, buffer);
        printf(">ds:%s,", buffer);
        floatToString(left_wheel.measured_speed_angular(), buffer);
        printf("rs:%s\n", buffer);
    }
}