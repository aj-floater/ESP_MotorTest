#ifndef ENCODER_H  // If ENCODER_H is not defined
#define ENCODER_H  // Define ENCODER_H

#include "mbed.h"

// Header file content goes here
class Encoder {
  public:

  InterruptIn ChanelA, ChanelB;
  Ticker Encoder_dt; 
  float dt = 0.1;
  volatile float EncoderTick;
  volatile int countA = 0, countB = 0;
  

  Encoder(PinName ChA, PinName ChB) : 
      ChanelA(ChA), 
      ChanelB(ChB)
  {
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

#endif // End of include guard