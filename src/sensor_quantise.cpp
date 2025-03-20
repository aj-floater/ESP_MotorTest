#include "mbed.h"
#include "stdio.h"
#include "display.h"
#include "read.h"

#define BUFFER_SIZE 20
#define no_signal 100

int main(){
    bool direction = 0;
    while (true) {
        float left_voltage = left_analog_sensor.read() * 3.3;  
        float right_voltage = right_analog_sensor.read() * 3.3;
    
        bool leftmost_value = leftmost_digital_sensor.read();
        bool left_inner_value = left_inner_digital_sensor.read();
        bool right_inner_value = right_inner_digital_sensor.read();
        bool rightmost_value = rightmost_digital_sensor.read();

        // Rolling buffer to handle short line breaks
        bool line_history[BUFFER_SIZE] = {true};

        bool left_detected = detect_white_line(left_analog_sensor);
        bool right_detected = detect_white_line(right_analog_sensor);
    
        //char left_buffer[10], right_buffer[10];
        //floatToString(left_voltage, left_buffer);
        //floatToString(right_voltage, right_buffer);

        bool linear = left_detected || right_detected;
        
        float diff = right_voltage - left_voltage;
        float sum = right_voltage + left_voltage;
        float spec = 0;

        bool lost = !(abs(spec) == no_signal);

        if(linear){
            if(sum > 0.38){
                spec = -(2.214 * sum + 1.645);
                if(diff < 0 ){
                    spec = -spec;
                }else{
                    spec = spec;
                }
            }else{
                spec = 0;
            }   
        }else if(!linear){
            if(leftmost_value){
                spec = -52;
                direction = 0;
            }else if(left_inner_value){
                spec = -27;
                direction = 0;
            }else if(right_inner_value){
                spec = 27;
                direction = 1;
            }else if(rightmost_value){
                spec = 52;
                direction = 1;
            }else{
                if(direction){
                    spec = no_signal; //Need Immediate turn right
                }else{
                    spec = - no_signal; //Need Immediate turn left
                }
            }
        }
        bool on_line = check_line_with_memory(lost, line_history, BUFFER_SIZE);

        char specbuffer[20];
        //char sumbuffer[20];
        //floatToString(sum,sumbuffer);
        floatToString(spec, specbuffer);
        //printf("%s\n",sumbuffer);
        //printf(">spec:%s\n", specbuffer);
        printf(">spec:%s", specbuffer);
        printf("on:%i\n",int(on_line));
    }
}