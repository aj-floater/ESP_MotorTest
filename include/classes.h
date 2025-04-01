#include "mbed.h" 
#include "read.h"



// This class create Encoders objects.
// Speed Linear and Angular can be measured.
class Encoder {

    public:
        InterruptIn ChanelA, ChanelB;
        Ticker Encoder_dt; 
    
        float dt = 0.01f;                // 10 ms ticker interval
        volatile float EncoderTick = 0;  // Encoder ticks-per-second on Channel A
        volatile int countA = 0, countB = 0;
        volatile bool direction = true;  // true = clockwise, false = counterclockwise
        bool status;                     // Used for inverting direction logic if needed
    
        // Low-pass filter variables
        // Hardcode alpha for a ~25 Hz cutoff at 100 Hz sample rate
        const float alpha = 0.15f; 
        volatile float filteredLinearSpeed = 0.0f;
    
        // Constructor
        Encoder(PinName ChA, PinName ChB, bool Inverted)
            : ChanelA(ChA), ChanelB(ChB), status(Inverted)
        {
        }
    
        // Setup interrupts
        void initialise(void) {
            // Attach the periodic interrupt (every dt seconds)
            Encoder_dt.attach(callback(this, &Encoder::EncoderCycleISR), dt);
    
            // Attach rising-edge interrupts for each channel
            ChanelA.rise(callback(this, &Encoder::ChanelA_countISR));
            ChanelB.rise(callback(this, &Encoder::ChanelB_countISR));
        }
    
        // Return the filtered linear speed
        float speed_linear(void) {
            return filteredLinearSpeed;
        }
    
        // Return the unfiltered angular speed
        float speed_angular(void) {
            // Convert ticks/s → rad/s
            float radPERseconds = (EncoderTick / 256.0f) * 2.0f * 3.141519f;
            if(!direction) {
                radPERseconds = -radPERseconds;
            }
            return radPERseconds;
        }
    
    protected:
        // Called every dt seconds
        void EncoderCycleISR(void) {
            // (1) Calculate pulses/sec on Channel A
            EncoderTick = static_cast<float>(countA) / dt;  
    
            // (2) Calculate raw linear speed
            //     rawSpeed (m/s) = (rad/s) * (wheel radius)
            //     where wheel radius = (0.078 m) * 0.5
            float rawSpeed = (EncoderTick / 256.0f) * 2.0f * 3.141519f * (0.078f * 0.5f);
    
            // Apply direction sign
            if(!direction) {
                rawSpeed = -rawSpeed;
            }
    
            // (3) Low-pass filter: y[n] = y[n-1] + alpha * (x[n] - y[n-1])
            filteredLinearSpeed += alpha * (rawSpeed - filteredLinearSpeed);
    
            // (4) Reset counts for next sampling window
            countA = 0;
            countB = 0;
        }
    
        void ChanelA_countISR(void) {
            countA++;
    
            // Determine direction by reading Channel B
            // status == true => normal logic
            // status == false => inverted logic
            if (status) {
                direction = (ChanelB.read() == 1) ? true : false;  
            } else {
                direction = (ChanelB.read() == 1) ? false : true; 
            }
        }
    
        void ChanelB_countISR(void) {
            countB++;
            // Not used for direction here, but could be expanded if needed.
        }
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
   
    float volatile prev_error, prev_input;
    float volatile output = 0.0f;
    float volatile error = 0.0f;

    float derivative_filtered;   
    float alpha;      

    public:
        float setpoint, Kp, Ki, Kd, max_out, min_out, freq;
        Integrator Integration;

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
                         alpha(0.2f),
                         derivative_filtered(0.0f){}
        
        void start(Callback<float()> input_func){
            _input_func = input_func;
            Integration.start(callback(this, &PID::get_error));
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



        void PID_Cycle_ISR(void) {
            // sample the input
            float input = _input_func.call();
            error = setpoint - input;
            
            
            // compute derivative
            float derivativeRaw = (error - prev_error) * freq; // multiply by freq if dt=1/freq
            prev_error = error;  // only update prev_error here
            prev_input = input;
        
            // integrate the error
            float integral = Ki * Integration.getIntegral();
            //  if (integral >= max_out)      Integration.reset();
            //  else if (integral <= min_out) Integration.reset();
        
            // P, I, D
            float proportional = Kp * error;
        
            // Low-pass filter the derivative if desired:
            derivative_filtered = alpha * derivative_filtered + (1.0f - alpha) * derivativeRaw;
        
            output = proportional + integral + Kd * derivative_filtered;
            
            // clamp output
            if (output >= max_out)      output = max_out;
            else if (output <= min_out) output = min_out;
        }
};







class MovingAverfiltergeFilter {
public:
    #define FILTER_WINDOW 8
    MovingAverfiltergeFilter() : index(0), sum(0.0f) {
        for (int i = 0; i < FILTER_WINDOW; ++i) buffer[i] = 0.0f;
    }

    float filter(float new_value) {
        sum -= buffer[index];
        buffer[index] = new_value;
        sum += new_value;
        index = (index + 1) % FILTER_WINDOW;
        return sum / FILTER_WINDOW;
    }

private:
    float buffer[FILTER_WINDOW];
    int index;
    float sum;
};

struct SpecResult {
    float spec;
    bool stop;
};

SpecResult LineDistance(MovingAverfiltergeFilter& left_filter, MovingAverfiltergeFilter& right_filter,  bool* line_history) {
    #define THRESHOLD 2.8
    #define BUFFER_SIZE 16
    #define no_signal 80
   
    float la = left_analog_sensor.read();
    float ra = right_analog_sensor.read();

    float lfilter = left_filter.filter(la);
    float rfilter = right_filter.filter(ra);

    bool lm = leftmost_digital_sensor.read();
    bool li = left_inner_digital_sensor.read();
    bool ri = right_inner_digital_sensor.read();
    bool rm = rightmost_digital_sensor.read();

    bool left_detected = (lfilter * 3.3 < THRESHOLD);
    bool right_detected = (rfilter * 3.3 < THRESHOLD);

    bool dactive = !(lm && li && ri && rm);
    bool linear = left_detected || right_detected;

    float diff = rfilter - lfilter;
    float sum = rfilter + lfilter;

    bool stop = memory(dactive, line_history, BUFFER_SIZE);

    float spec = 0.0f;
    bool direction = 0;

    char bufferA[20];
    char bufferB[20];

    //floatToString(lfilter, bufferA);
    //floatToString(rfilter, bufferB);

    //printf("LM=%d, LI=%d, RI=%d, RM=%d\n",lm, li, ri, rm);
 

    if (linear) {
        if (sum < 0.38f) {
            spec = 0.0f;
        } else {
            float dist_eqn = 2.3129f * 3.3 * sum + 2.2961f;
            spec = diff > 0 ? dist_eqn : -dist_eqn;
        }
        /*printf("Linear\n");*/
    } else {
        /*printf("NOT LINEAR\n");*/
        if (!lm)      { spec = 52.0f; direction = 0; /*printf("NOT LM\n");*/}
        else if (!li) { spec = 27.0f; direction = 0; /*printf("NOT LI\n");*/}
        else if (!ri) { spec = -27.0f; direction = 1; /*printf("NOT RI\n");*/}
        else if (!rm) { spec = -52.0f; direction = 1; /*printf("NOT RM\n");*/}
        else {
            /*printf("ALL 1\n");*/
            if (stop) {
                /*printf("SHOULD BE 1\n");*/
                return {spec, true};
            }
        }
    }
    /*printf("SHOULD BE 0\n");*/
    return {spec, false};

}