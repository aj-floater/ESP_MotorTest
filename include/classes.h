#include "mbed.h" 

class Encoder {

    public:

    InterruptIn ChanelA, ChanelB;
    Ticker Encoder_dt; 
    float dt = 0.01;
    volatile float EncoderTick;
    volatile int countA = 0, countB = 0;
    volatile bool direction = true;  // true = clockwise, false = counterclockwise
    bool status;
    

    Encoder(PinName ChA, PinName ChB, bool Inverted) : ChanelA(ChA), ChanelB(ChB), status(Inverted){}

    void initialise(void){
        
        Encoder_dt.attach(callback(this,&Encoder::EncoderCycleISR), dt);

        
        ChanelA.rise(callback(this,&Encoder::ChanelA_countISR));
        ChanelB.rise(callback(this,&Encoder::ChanelB_countISR));

    }

    float speed_linear(void){
        float radPERseconds = (EncoderTick / 256.0f) * 2.0f * 3.141519f;
        float wheelVelocity = 0.078f * 0.5f * radPERseconds;

        if(!direction){wheelVelocity = -1.0f * wheelVelocity;}
        return wheelVelocity;
    }
    float speed_angular(void){
        float radPERseconds = (EncoderTick / 256.0f) * 2.0f * 3.141519f;

        if(!direction){radPERseconds = -1.0f * radPERseconds;}
        return radPERseconds;
    }

    protected:

    void EncoderCycleISR(void){
        EncoderTick = (float(countA)/dt);
        countA = 0; countB = 0;
    }

    void ChanelA_countISR(void){
        countA++; 
        switch(status)
        {
        case true:
            if(ChanelB.read()==1){direction = false;}else{direction = true;}
            break;
        case false: 
            if(ChanelB.read()==1){direction = true;}else{direction = false;}
            break;
        
        default:
            break;
        }
        
    }
        
    void ChanelB_countISR(void){countB++;}
};

// class Wheel {
//     private:
//     public:
//         float desired_speed;
//         float measured_speed;
    
//         float proportional_gain;
//         float previous_error;
    
//         float control_output;
    
//         Encoder encoder;
//         PwmOut motor;
    
//         Wheel(float Kp, PinName ChA, PinName ChB, PinName pwm, float frequency) : 
//             proportional_gain(Kp), 
//             //encoder(ChA, ChB, ),
//            // motor(pwm)
//         {
//             encoder.initialise();
    
//             motor.period_us(45);
//             motor.write(1.0f);
    
//             desired_speed = 0.0f;
//             measured_speed = 0.0f;
//             control_output = 1.0f;
//         }
    
//         float measured_speed_angular() { return encoder.speed_angular(); };
//         float measured_speed_linear() { return encoder.speed_linear(); };
    
//         // Set speed in rad/s
//         float speed(float s){
//             desired_speed = s;
//             return 0;
//         }
//         float speed(){
//             return desired_speed;
//         }
    
//         // Calculate the error e(t) = Desired Speed − Measured Speed
//         float error(){
//             return measured_speed_angular() - desired_speed;
//         }
    
//         // Calculate the control output (ie the PWM duty cycle)
//         void pControl(){
//             float Et = error(); // gets the error at the current time
    
//             float proportional_term = proportional_gain * Et;
//             control_output += proportional_term;
    
//             // Setup values for next iteration
//             previous_error = Et;
//         }
    
//         void update(){
//             // Clamp final output to 0.0f and 1.0f
//             // pControl();
//             control_output = 1 - desired_speed/50;
    
//             if (control_output < 0.0f) control_output = 0.0f;
//             else if (control_output > 1.0f) control_output = 1.0f;
    
//             motor.write(control_output);
//         }
//     };

    // This class creates Integrator objects. 
    // Why? the code will be long and throughout it we will certainly need to perform simultaneous integrations
    //      and perhaps they need to have different frequencies.
    // How? it will make use of the Ticker API from Mbed to perform the integration. 
    // Note: parameter of the start member function must be a pointer to a function (was not covered in MCEII)
    //       , and it has to have type float and no parameters.
    // If you face problems using this class, ask me (Hamed) :)
class Integrator {
    public:
        //The frequency (in Hz) at which to update the integral.
        Integrator(float update_freq)
            : _update_freq(update_freq), _integral(0.0f) {
        }
    
        /*
         * Starts the integrator.
         *
         * This can be a pointer to a free function or to a member function.
         * For example:
         *     start(callback(&someObject, &SomeClass::methodReturningFloat))
         * or
         *     start(callback(freeFunctionReturningFloat))
         */
        void start(Callback<float()> input_func) {
            _input_func = input_func;
            // Attach our periodic update method at the specified frequency
            _ticker.attach(callback(this, &Integrator::updateISR), 1.0f / _update_freq);
        }
    
        //Stops the integrator (detaches the Ticker).
        void stop() {
            _ticker.detach();
        }
    
        //@brief Returns the current integral value.
        float getIntegral() const {
            return _integral;
        }

        void reset(void){
            _integral = 0;
        }
    
    private:
        //Periodic update function called by the Ticker.
        void updateISR() {
            if (_input_func) {
                // Get the input and integrate using dt = 1/freq
                float value = _input_func();
                _integral += value * (1.0f / _update_freq);
            }
        }
    
    private:
        Ticker           _ticker;      // Ticker for periodic callbacks
        Callback<float()> _input_func; // Function that provides the value to integrate
        float            _update_freq; // Integration frequency (Hz)
        volatile float            _integral;    // Accumulated integral
    };