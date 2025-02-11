#include "mbed.h"
#include "C12832.h"

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

    float speed(void){
        float radPERseconds = (EncoderTick / 256.0f) * 2.0f * 3.141519f;
        float wheelVelocity = 0.08f * radPERseconds;
        return wheelVelocity;
    }

    protected:

    void EncoderCycleISR(void){

        EncoderTick = (float(countA)/dt);
        countA = 0; countB = 0;
    }

    void ChanelA_countISR(void){countA++;}
    void ChanelB_countISR(void){countB++;}
};

void floatToString(float value, char *buffer) {
    int int_part = (int)value;  // Extract integer part
    float decimal_part = value - int_part;  // Get fractional part

    if (decimal_part < 0) decimal_part = -decimal_part;  // Handle negatives
    int decimal_int = (int)(decimal_part * pow(10, 3) + 0.5); // Scale & round

    // Format as string
    sprintf(buffer, "%d.%.3d", int_part, decimal_int);
}


int main(void){
    // ----------------------------
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
    // ----------------------------

    Encoder Encoder1(PA_12, PA_11);
    Encoder1.initialise();

    // ----------------------------

    C12832 lcd(D11, D13, D12, D7, D10);


    // ----------------------------
    Motor1.period_us(45);
    Motor2.period_us(45);

    Motor1.write(0.6f);
    Motor2.write(0.6f);

    char buffer[50];

    float speed1 = 0.2f;
    float speed2 = 0.4f;
    float speed3 = 0.6f;
    float speed4 = 0.8f;

    int i = 0;

    while(1){

        // Speed Changes Section
        if (i < 20){
            Motor1.write(speed1);
            Motor2.write(speed1);
        }
        else if (i < 40){
            Motor1.write(speed2);
            Motor2.write(speed2);
        }
        else if (i < 60){
            Motor2.write(speed3);
            Motor1.write(speed3);
        }
        else if (i < 80){
            Motor1.write(speed4);
            Motor2.write(speed4);
        }
        
        i++;
        if (i > 80){
            i = 0;
        }

        // Display Section
        float speed = Encoder1.speed();
        lcd.locate(10,0);
        floatToString(speed, buffer);
        lcd.printf("Converted String: %s\n", buffer);
        
        lcd.locate(10, 20);
        lcd.printf("i: %d", i);

        wait_us(1000);
    }
}