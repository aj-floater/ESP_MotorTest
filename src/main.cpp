#include "mbed.h"
#include "C12832.h"
#include "classes.h"
#include "functions.h"

enum class State : uint8_t {
    Turn            = 'T',
    LineFollowing   = 'L',
    Controller      = 'C',
    ChangeValue     = 'V',
    Idle            = 'I',
    None            = 0
};

// Holds the current state; starts in LineFollowing by default
volatile State currentState = State::LineFollowing;

void pollBLEState(HM10 &hm10) {
    int r = hm10.read();
    if (r >= 0) {
        char cmd = hm10.currentReadBuffer[1];
        switch(cmd) {
            case 'T': currentState = State::Turn;           break;
            case 'L': currentState = State::LineFollowing;  break;
            case 'C': currentState = State::Controller;     break;
            case 'V': currentState = State::ChangeValue;    break;
            case 'I': currentState = State::Idle;           break;
            default:  /* ignore unknown */                  break;
        }
    }
}


volatile float line_dist = 0.0f;
volatile bool stop_state = true;

bool line_history[BUFFER_SIZE] = {false};
MovingAverfiltergeFilter left_filter;
MovingAverfiltergeFilter right_filter;

void updateLineDistance() {
    // read the sensors in a thread-safe context
    SpecResult result = LineDistance(left_filter, right_filter, line_history);
    line_dist = - result.spec;
    stop_state = result.stop;
}

// The PID won't call LineDistance() from ISR anymore.
// Instead, it calls a simple function that just returns 'line_dist'.
float lineDistGetter() {
    return line_dist;
}

bool stopStateGetter() {
    return stop_state;
}

int main(void){

    // Pin Setup
        DigitalOut Bipolar1(PB_13);
        Bipolar1.write(0);

        DigitalOut Direction1(PB_14);
        Direction1.write(1);
        
        PwmOut Motor1(PC_6);

        DigitalOut Bipolar2(PB_15);
        Bipolar2.write(0);

        DigitalOut Direction2(PB_1);
        Direction2.write(1);

        PwmOut Motor2(PC_8);

        DigitalOut Enable(PB_2);
        Enable.write(1);
    

        // Encoders setup
        Encoder Encoder1(PC_2,PC_3,0);   //right
        Encoder Encoder2(PC_7,PA_9,1);   //left

        Encoder1.initialise();
        Encoder2.initialise();

        // Integrator setup
        Integrator I1(1000.0f);
        I1.start(callback(&Encoder1,&Encoder::speed_linear));
        // Integrator I2(1000.0f);
        // I2.start(callback(&Encoder2,&Encoder::speed_linear));

        Integrator I3(1000.0f);
        auto RobotAngularV_Callback = [&]() -> float {
            return RobotAngularV(Encoder1, Encoder2);
        };
        I3.start(RobotAngularV_Callback);

        // float issue
        char buffer1[50];
        char buffer2[50];

        float distance;

        float setspeed = 0.6f;

        // PID
        PID Right(setspeed, 1.8f, 4.9f, 0.0f, 4.0f, -4.0f, 3300); //float setpoint,float Kp,float Ki,float Kd,float max_out,float min_out,float freq
        PID Left(setspeed, 1.8f, 4.9f, 0.0f, 4.0f, -4.0f, 3300); // p=1.8, i=4.5, d=0

        PID Position(0.0f, 0.3f, 0.5f, 0.0f, 300.0f, -300.0f, 3300);// s= 0.6 - kp = 0.3, ki = 0.5 with k=7 and divided by 52 bounds at 152

        EventQueue queue;
        Thread queueThread;
        queueThread.start(callback(&queue, &EventQueue::dispatch_forever));

        // For example, update line_dist at 1000 Hz in the queue
        queue.call_every(1ms, updateLineDistance);

        // Meanwhile, the PID Ticker runs at 3300 Hz in ISR context,
        // but only calls 'lineDistGetter()', which doesn't lock a mutex.
        Position.start(callback(lineDistGetter));

        Left.start(callback(&Encoder2,&Encoder::speed_linear));
        Right.start(callback(&Encoder1,&Encoder::speed_linear));


        float position;


        char linear1_buffer[20];
        char linear2_buffer[20];

        Timer t;

        Motor1.write(1.0f);
        Motor2.write(1.0f);

        stop_state = 0;
        HM10 hm10(PA_11, PA_12);


    while(1){
        while (stop_state == 0) {
            pollBLEState(hm10);
            switch(currentState) {
                case State::Turn:
                    // run your Turn(...) once, then switch back to LineFollowing
                    Turn(-3.4f, Motor1, Motor2, I3, Direction1, Direction2);
                    currentState = State::LineFollowing;
                    break;
        
                case State::LineFollowing:
                    // normal line-following
                    FollowLine(Left, Right, Position, Motor1, Motor2, setspeed);
                    break;

                case State::Controller:
                    // controlled using computer

                    break;
        
                case State::ChangeValue: {
                    // Read any new BLE packet
                    if (hm10.read() >= 0) {
                        // Expect format: "V,<name>,<value>"
                        char* buf = hm10.currentReadBuffer;
                        // strtok will split on commas:
                        char* token = strtok(buf, ",");      // token == "V"
                        char* varName = strtok(nullptr, ","); // e.g. "setspeed"
                        char* valStr  = strtok(nullptr, ","); // e.g. "0.75"
        
                        if (varName && valStr) {
                            float newVal = atof(valStr);
        
                            // Match against your known variables
                            if (strcmp(varName, "setspeed") == 0) {
                                setspeed = newVal;
                            }
                            else if (strcmp(varName, "Position.Kp") == 0) {
                                Position.Kp = newVal;
                            }
                            else if (strcmp(varName, "Position.Ki") == 0) {
                                Position.Ki = newVal;
                            }
                            else if (strcmp(varName, "Position.Kd") == 0) {
                                Position.Kd = newVal;
                            }
                        }
                    }
                    break;
                }

                case State::Idle:
                    break;
        
                default:
                    // No valid state yet—just idle or fallback:
                    break;
            }
        }        


        Motor1.write(1.0f);
        Motor2.write(1.0f);
        while(1){}
    }
}
