#ifndef ENCODER_H  // If ENCODER_H is not defined
#define ENCODER_H  // Define ENCODER_H

#include "mbed.h"

// Header file content goes here
class Encoder {

    public:

    InterruptIn ChanelA, ChanelB;
    Ticker Encoder_dt; 
    float dt = 0.01;
    volatile float EncoderTick;
    volatile int countA = 0, countB = 0;
    volatile bool direction = true;  // true = clockwise, false = counterclockwise
    bool status;
    

    Encoder(PinName ChA, PinName ChB, bool Inverted) : ChanelA(ChA), ChanelB(ChB), status(Inverted){
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
            if(ChanelB.read()==1) 
                direction = false;
            else
                direction = true;
            break;
        case false: 
            if(ChanelB.read()==1) 
                direction = true;
            else
                direction = false;
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

float RobotAngularV(Encoder& Encoder1, Encoder& Encoder2) {
    float encoder_difference = Encoder1.speed_linear() - Encoder2.speed_linear();
    float RobotAngularSpeed = encoder_difference / 0.173f;
    
    return RobotAngularSpeed;
}

#endif // End of include guard