#include "mbed.h"
#include "C12832.h"
#include "classes.h"
#include "functions.h"

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

        float setspeed = 0.9f; //0.6

        // PID
        PID Right(setspeed, 1.8f, 4.9f, 0.0f, 4.0f, -4.0f, 3300); //float setpoint,float Kp,float Ki,float Kd,float max_out,float min_out,float freq
        PID Left(setspeed, 1.8f, 4.9f, 0.0f, 4.0f, -4.0f, 3300); // p=1.8, i=4.5, d=0

        PID Position(0.0f, 0.158f, 0.25f, 0.06f, 10000.0f, -10000.0f, 3300);// s= 0.0 - kp = 0.3, ki = 0.3, kd = 0.05 with k=7 and divided by 52 bounds at 152

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

       do{
            if (hm10.read() != -EAGAIN) {
                hm10.write(hm10.currentReadBuffer);
                if (hm10.currentReadBuffer[0] == '1'){


                    Turn(-3.4, Motor1, Motor2, I3, Direction1, Direction2);

                    Left.Integration.reset();
                    Right.Integration.reset();
                    Position.Integration.reset();

                    stop_state =0;
                }
                if (hm10.currentReadBuffer[0] == '2'){
                    Right.setpoint = 0.3f;
                    Left.setpoint = 0.3f;
                }
                if (hm10.currentReadBuffer[0] == '3'){
                    Right.setpoint = 0.9f;
                    Left.setpoint = 0.9f;
                }
                if (hm10.currentReadBuffer[0] == '4'){
                    Right.setpoint = 1.2f;
                    Left.setpoint = 1.2f;
                }
            }

            FollowLine(Left, Right, Position, Motor1, Motor2, setspeed);
            printf("INSIDE DO LOOP \n");

            
        }while(stop_state == 0);


        Motor1.write(1.0f);
        Motor2.write(1.0f);
        while(1){}













        // distance = LineDistance();

        // floatToString(lineDistGetter(), buffer1);
         //floatToString(Encoder2.speed_linear(), buffer2);


         //printf("StopState= %s\n",buffer1 );
              // printf("StopState= %i\n",stop_state);
         


        // float linear1 =  Encoder2.speed_linear();//Encoder1.speed_linear();
        // float linear2 =  setspeed;
        // floatToString(linear1, linear1_buffer);
        // printf(">Position:%s\n", linear1_buffer);
        // floatToString(linear2, linear2_buffer);
        // printf(">PIDoutput:%s\n", linear2_buffer);
    }
}
