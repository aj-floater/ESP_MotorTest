#include "mbed.h" 


// This class create Encoders objects.
// Speed Linear and Angular can be measured.
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
            if(ChanelB.read()==1){direction = true;}else{direction = false;}
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



    // This class will create PID objects.
    // It can be used to perform PID control on any measured quantity.
class PID{
    private:
    Ticker PID_Cycle, error_Cycle;
    Callback<float()> _input_func;
    Integrator Integration;
    float volatile prev_error;
    float volatile output = 0.0f;
    float volatile error = 0.0f;

    float derivative_filtered;   
    float alpha;      

    public:
        float setpoint, Kp, Ki, Kd, max_out, min_out, freq;

        
        PID(float setpoint,
             float Kp,
              float Ki,
               float Kd,
                float max_out,
                 float min_out,
                  float freq)
                  :Kp(Kp),
                   Ki(Ki),
                    Kd(Kd),
                     max_out(max_out),
                      min_out(min_out),
                       freq(freq),
                        setpoint(setpoint),
                         Integration(freq * 10.0f),
                         alpha(0.9f),
                         derivative_filtered(0.0f){}
        
        void start(Callback<float()> input_func){
            _input_func = input_func;
            Integration.start(callback(this, &PID::get_error));
            error_Cycle.attach(callback(this, &PID::error_ISR), 1.0f / (freq * 10.0f));
            PID_Cycle.attach(callback(this, &PID::PID_Cycle_ISR), 1.0f / freq);
        }

        float get_error(void){
            return error;
        }

        float get_output(void){
            return output;
        }

        void stop() {
            PID_Cycle.detach();
            error_Cycle.detach();
            Integration.stop();
        }

    protected:


        void error_ISR(void){
            prev_error = error;
            error = setpoint - _input_func.call();
        }

        void PID_Cycle_ISR(void){

            float proportional = Kp * error;

            float integral = Ki * Integration.getIntegral();

            float derivativeRaw = ((error - prev_error)/(1.0f/(freq*10.0f)));
            derivative_filtered = alpha * derivative_filtered + (1.0f - alpha) * derivativeRaw;
            float derivative = Kd * derivative_filtered;

            output = proportional + integral + derivative;

            if (output >= max_out)
            {
                output = max_out;
            } else if (output <= min_out)
            {
                output = min_out;
            }     

        }
};