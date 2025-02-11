#include "mbed.h"
#include "C12832.h"

class Encoder {

    public:

    InterruptIn ChanelA, ChanelB;
    Ticker Encoder_dt; 
    float dt = 1;
    volatile float EncoderTick;
    volatile int countA = 0, countB = 0;
    

    Encoder(PinName ChA, PinName ChB) : ChanelA(ChA), ChanelB(ChB){}

    void initialise(void){
        
        Encoder_dt.attach(callback(this,&Encoder::EncoderCycleISR), dt);
        
        ChanelA.rise(callback(this,&Encoder::ChanelA_countISR));
        ChanelB.rise(callback(this,&Encoder::ChanelB_countISR));

    }

    float speed(void){
        
        float radPERseconds = (EncoderTick/256.0)*2.0*3.141519;
        float wheelVelocity = 0.08*radPERseconds;
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

    // Motor1.write(0.6f);
    // Motor2.write(0.6f);

    while(1){

        float speed = Encoder1.speed();
        lcd.locate(10,0);
        lcd.printf("Wheel speed is: %d \n", speed);
       // wait_us(100000);
    }
}