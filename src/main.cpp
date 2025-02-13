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
    float desired_speed;
    float measured_speed;

    float proportional_gain;
    float integral_gain;
    float derivative_gain;

    float control_output;

    Timer timer;
    float previous_time;
    float previous_error;
    float previous_integral;

    Encoder encoder;
    PwmOut motor;

public:
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

        desired_speed = 0.0f;
        measured_speed = 0.0f;
        control_output = 0.6f;
    }

    float measured_speed_angular() { return encoder.speed_angular(); };
    float measured_speed_linear() { return encoder.speed_linear(); };

    float deltaTime(){
        int dt = timer.read_us() - previous_time;
        timer.reset();
        return dt / 1'000'000.0f;  // Convert µs to seconds
    }

    float integral(float dt, float Et){
        float current_integral = previous_integral + (dt/2) * (Et + previous_error);
        
        // Setup values for next iteration
        previous_integral = current_integral;
        return current_integral;
    }

    float derivative(float dt, float Et){
        return (Et - previous_error) / dt;
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
        return desired_speed - measured_speed_angular();
    }

    // Calculate the control output (ie the PWM duty cycle)
    void pidControl(){
        float Et = error(); // gets the error at the current time
        float dt = deltaTime();

        float proportional_term = proportional_gain * Et;
        float integral_term = integral_gain * integral(dt, Et);
        float derivative_term = derivative_gain * derivative(dt, Et);

        control_output = proportional_term + integral_term + derivative_term;

        // Setup values for next iteration
        previous_error = Et;
    }

    void update(){
        // Clamp final output to 0.0f and 1.0f
        float final_output;

        if (control_output < 0.0f) final_output = 0.0f;
        else if (control_output > 1.0f) final_output = 1.0f;
        else final_output = control_output;

        motor.write(final_output);
    }
};


//This code is invalid yet.
class Integrator {

    public:

    float f;
    Ticker I_period;
    volatile float summation = 0;
    float *signal;

    Integrator(float frequency, float* Ut):f(frequency), signal(Ut){
        I_period.attach(callback(this,&Integrator::IntegratorCycleISR),1.0f/f);
    }

    float get_integral(void){
        return summation;
    }


    protected:
    
    void IntegratorCycleISR(void){
            summation = (1.0f/f)*(*signal) + summation;
    }

};

void floatToString(float value, char *buffer) {
    int int_part = (int)value;  // Extract integer part
    float decimal_part = value - int_part;  // Get fractional part

    if (decimal_part < 0) decimal_part = -decimal_part;  // Handle negatives
    int decimal_int = (int)(decimal_part * pow(10, 3) + 0.5); // Scale & round

    // Format as string
    sprintf(buffer, "%d.%.3d", int_part, decimal_int);
}

Wheel left_wheel(1.0f, 0.0f, 0.0f, PA_12, PA_11, PC_6, 1.0f);

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

    C12832 lcd(D11, D13, D12, D7, D10);

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

    char buffer[50];
    left_wheel.speed(20.0f);

    //--------------------
     Encoder Encoder1(PA_12, PA_11);
     float integration =0;

    while(1){


        //left_wheel.update();

        
        left_wheel.update();


        // Display Section


        // lcd.locate(0,0);
        // //lcd.printf("Hello World");
        // floatToString(Kp, buffer);
        // lcd.printf("Kp: %s\n", buffer);
        // lcd.locate(0,10);
        // floatToString(left_wheel.speed(), buffer);
        // lcd.printf("s: %s\n", buffer);

        // float speed_l = Encoder1.speed_linear();
        // float speed_a = Encoder1.speed_angular();
        // floatToString(speed_l, buffer);
        // lcd.printf("LinearSpeed: %s\n", buffer);
        // lcd.locate(10, 10);
        // floatToString(speed_a, buffer);
        // lcd.printf("AngularSpeed: %s\n", buffer);
        // lcd.locate(10, 20);
        // lcd.printf("i: %d", i);


        float speed = Encoder1.speed_linear();
        lcd.locate(0,0);
        floatToString(speed, buffer);
        lcd.printf("Speed: %s\n", buffer);




        wait_us(1000);
    }
}