#include "mbed.h" 

class Encoder {

    public:

    InterruptIn ChanelA, ChanelB;
    Ticker Encoder_dt; 
    float dt = 0.1;
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
    
        Wheel(float Kp, PinName ChA, PinName ChB, PinName pwm, float frequency) : 
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
            control_output = 1 - desired_speed/50;
    
            if (control_output < 0.0f) control_output = 0.0f;
            else if (control_output > 1.0f) control_output = 1.0f;
    
            motor.write(control_output);
        }
    };

    // This class creates Integrator objects. 
    // Why? the code will be long and throughout it we will certainly need to perform simultaneous integrations
    //      and perhaps they need to have different frequencies.
    // How? it will make use of the Ticker API from Mbed to perform the integration. 
    // Note: parameter of the start member function must be a pointer to a function (was not covered in MCEII)
    //       , and it has to have type float and no parameters.
    // If you face problems using this class, ask me (Hamed) :)
class Integrator{

    private:

    float frequency;
    Ticker I_cycle;
    float volatile summation = 0;

    public:

        float (*FunctionPointer)(void);
        void* ObjectPtr = nullptr;  // Generic object pointer for member function
        float (Integrator::*MemberFunctionPointer)(void) = nullptr;  // Member function pointer

        Integrator(float freq):frequency(freq){};

        void start(float (*FcnPtr)(void)) {
            FunctionPointer = FcnPtr;  // Assign function pointer before using it
            
            if (FunctionPointer == nullptr) {
                printf("ERROR: FunctionPointer is NULL in start()!\n");
            } else {
                printf("FunctionPointer assigned: %p\n", FunctionPointer);
                I_cycle.attach(callback(this, &Integrator::UpdateISR), 1.0f / frequency);
            }
        }

            // Overloaded start() for member function pointers
        template <typename T>
        void start(T* obj, float (T::*FcnPtr)(void)) {
            if (!obj || !FcnPtr) {
            printf("ERROR: FunctionPointer is NULL in start()!\n");
            return;
            }
            ObjectPtr = obj;
            MemberFunctionPointer = reinterpret_cast<float (Integrator::*)(void)>(FcnPtr);
            FunctionPointer = nullptr;  // Disable normal function pointer

            I_cycle.attach(callback(this, &Integrator::UpdateISR), 1.0f / frequency);
        }

        void reset(void){
            summation = 0;
        }



        void stop(void){
            I_cycle.detach();
        }
        

        float getvalue(void){
            return summation;
        }

    protected:

        void UpdateISR(void){
            summation = summation + (1.0f/frequency)*FunctionPointer();
        }
};