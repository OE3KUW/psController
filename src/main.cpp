/******************************************************************
                        LoLin32 psController 
                                                    қuran nov 2023
******************************************************************/
#include <Arduino.h>
#include <PS4Controller.h>

#define TRUE                            true
#define FALSE                           false
#define WAIT_ONE_SEC                    10000
#define H                               HIGH
#define L                               LOW

#define WHEEL_L                         2
#define WHEEL_R                         A4
#define WHEEL_L_DIRECTION               A5
#define WHEEL_R_DIRECTION               15



volatile int oneSecFlag;
hw_timer_t *timer = NULL;
void IRAM_ATTR myTimer(void);

volatile int vL, vR;


void setup() 
{
    Serial.begin(115200);
    Serial.println("start!");

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &myTimer, true);
    timerAlarmWrite(timer, 100, true);  // 0.1 msec
    timerAlarmEnable(timer);
    
    oneSecFlag = FALSE; 
    PS4.begin("10:20:30:40:50:62");
    Serial.println("ready!");

    digitalWrite(WHEEL_L, L);


    pinMode(WHEEL_L, OUTPUT);
    pinMode(WHEEL_R, OUTPUT);
    pinMode(WHEEL_L_DIRECTION, OUTPUT);
    pinMode(WHEEL_R_DIRECTION, OUTPUT);

    digitalWrite(WHEEL_L_DIRECTION, L);
    digitalWrite(WHEEL_R_DIRECTION, H);

    digitalWrite(WHEEL_L, L); // stop !
    digitalWrite(WHEEL_R, L); // stop !

    //digitalWrite(WHEEL_L, H); // go !
    //digitalWrite(WHEEL_R, H); // go !

    vL = vR = 0;


}
void loop() 
{
  uint8_t leftstickXDATA0 = 0;
  uint8_t leftstickXDATA1 = 0;
  static int t1 = 0;

    if (oneSecFlag)
    {
        oneSecFlag = FALSE;

        if(PS4.isConnected())
        {
            //Serial.println("connected!");
            if(PS4.Triangle()) Serial.println("Triangle!"); 
            if(PS4.Up()) Serial.println("Up!");
            if(PS4.L1()) Serial.println("L1!"); 

            //Serial.print(PS4.L2Value());
            //Serial.print(PS4.R2Value());
            //Serial.print(PS4.GyrX());
            //Serial.println(PS4.GyrY());
            
            vL = PS4.L2Value();
            vR = PS4.R2Value();
   

        }
        else
        {
             //Serial.print("*");
        }
    }    
}    

//** Timer Interrupt:   ******************************************

void IRAM_ATTR myTimer(void)   // periodic timer interrupt, expires each 0.1 msec
{
    static int32_t tick = 0;
    static unsigned char ramp = 0;
    
    tick++;
    ramp++;

    if (tick >= WAIT_ONE_SEC) 
    {
        oneSecFlag = TRUE;
        tick = 0; 
    }

    if (ramp > vL) digitalWrite(WHEEL_L, L);  else digitalWrite(WHEEL_L, H);
    if (ramp > vR) digitalWrite(WHEEL_R, L);  else digitalWrite(WHEEL_R, H);

}


