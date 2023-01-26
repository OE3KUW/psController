/******************************************************************
                        LoLin32 psController
                                                    қuran jan 2023
******************************************************************/
#include <Arduino.h>
#include <PS4Controller.h>

#define TRUE                             true
#define FALSE                            false
#define WAIT_ONE_SEC                     10000

volatile int oneSecFlag;
hw_timer_t *timer = NULL;
void IRAM_ATTR myTimer(void);

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
}
void loop() 
{
    static int t1 = 0;
    if (oneSecFlag)
    {
        oneSecFlag = FALSE;

        if(PS4.isConnected())
        {
            Serial.println("connected!");
            if(PS4.Triangle()) Serial.println("Triangle!");
            if(PS4.Up()) Serial.println("Up!");
        }
        else
        {
             Serial.print("*");
        }
    }    
}    

//** Timer Interrupt:   ******************************************

void IRAM_ATTR myTimer(void)   // periodic timer interrupt, expires each 0.1 msec
{
    static int32_t tick = 0;
    
    tick++;

    if (tick >= WAIT_ONE_SEC) 
    {
        oneSecFlag = TRUE;
        tick = 0; 
    }
}


