/******************************************************************
                        LoLin32 psController 
                                                    қuran nov 2023
******************************************************************/
#include <Arduino.h>
#include <PS4Controller.h>

#define FASTLED_ALL_PINS_HARDWARE_SPI
#include <FastLED.h>

#define TRUE                            true
#define FALSE                           false
#define WAIT_ONE_SEC                    10000
#define WAIT_ONE_10MSEC                 100

#define H                               HIGH
#define L                               LOW

#define ON_BOARD_LED                    5

#define NUM_LEDS                        4
#define DATA_PIN                        23
#define CLOCK_PIN                       18


#define WHEEL_L                         2
#define WHEEL_R                         A4
#define WHEEL_L_DIRECTION               A5 
#define WHEEL_R_DIRECTION               15



volatile int oneSecFlag;
volatile int tenMSecFlag;

hw_timer_t *timer = NULL;
void IRAM_ATTR myTimer(void);

volatile int vL, vR;

CRGB leds[NUM_LEDS];

volatile int pulse; 
volatile int connected;
volatile int LDir;
volatile int RDir;


void setup() 
{
    Serial.begin(115200);
    Serial.println("start!");

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &myTimer, true);
    timerAlarmWrite(timer, 100, true);  // 0.1 msec
    timerAlarmEnable(timer);
    
    oneSecFlag = FALSE; 
    tenMSecFlag = FALSE; 
    PS4.begin("10:20:30:40:50:62");
    Serial.println("ready!");

    pinMode(ON_BOARD_LED, OUTPUT);

    pinMode(WHEEL_L, OUTPUT);
    pinMode(WHEEL_R, OUTPUT);
    pinMode(WHEEL_L_DIRECTION, OUTPUT);
    pinMode(WHEEL_R_DIRECTION, OUTPUT);

    digitalWrite(ON_BOARD_LED, L); // on ! ... blue 

    digitalWrite(WHEEL_L_DIRECTION, L);
    digitalWrite(WHEEL_R_DIRECTION, H);

    digitalWrite(WHEEL_L, L); // stop !
    digitalWrite(WHEEL_R, L); // stop !

    //digitalWrite(WHEEL_L, H); // go !
    //digitalWrite(WHEEL_R, H); // go !

    pulse = 0;
    connected = 0;
    LDir = 0;
    RDir = 1;

    vL = vR = 0;

// Fast Leds:

    FastLED.addLeds<SK9822, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
    
    leds[0] = CRGB{255, 255, 255}; // R B G
    leds[1] = CRGB{255, 255, 255};
    leds[2] = CRGB{255, 255, 255};
    leds[3] = CRGB{255, 255, 255};

    FastLED.show();





}
void loop() 
{
  uint8_t leftstickXDATA0 = 0;
  uint8_t leftstickXDATA1 = 0;
  static int t1 = 0;

    if (oneSecFlag)
    {
        oneSecFlag = FALSE;

        pulse = pulse ? 0 : 1;

        digitalWrite(ON_BOARD_LED, pulse);


        if(PS4.isConnected())
        {
            connected = 1;
            digitalWrite(ON_BOARD_LED, pulse);  // off

            leds[0] = CRGB{0, 0, 255}; // R B G
            leds[1] = CRGB{0, 255, 0};
            leds[2] = CRGB{255, 0, 255};  // Yellow: FF00FF
            leds[3] = CRGB{255, 0, 0};

            FastLED.show();




            //Serial.println("connected!");
            if(PS4.Triangle()) Serial.println("Triangle!"); 
            if(PS4.Up()) Serial.println("Up!");


            //Serial.print(PS4.L2Value());
            //Serial.print(PS4.R2Value());
            //Serial.print(PS4.GyrX());
            //Serial.println(PS4.GyrY());
            
            //vL = PS4.L2Value();
            //vR = PS4.R2Value();
        }
        else
        {
         
            

             //Serial.print("*");
        }

      

    }    

    if (tenMSecFlag)
    {
        // "entprellt" 

        if(PS4.L1()) {LDir = LDir ? 0 : 1;  digitalWrite(WHEEL_L_DIRECTION, LDir);}
        if(PS4.R1()) {RDir = RDir ? 0 : 1;  digitalWrite(WHEEL_R_DIRECTION, RDir);}

    }

    if (connected)
    {
        vL = PS4.L2Value();
        vR = PS4.R2Value();


    }

}    

//** Timer Interrupt:   ******************************************

void IRAM_ATTR myTimer(void)   // periodic timer interrupt, expires each 0.1 msec
{
    static int32_t tick = 0;
    static int32_t mtick = 0;
    static unsigned char ramp = 0;
    
    tick++;
    mtick++;
    ramp++;

    if (tick >= WAIT_ONE_SEC) 
    {
        oneSecFlag = TRUE;
        tick = 0; 
    }

    if (mtick >= WAIT_ONE_10MSEC) 
    {
        tenMSecFlag = TRUE;
        mtick = 0; 
    }

    if (ramp > vL) digitalWrite(WHEEL_L, L);  else digitalWrite(WHEEL_L, H);
    if (ramp > vR) digitalWrite(WHEEL_R, L);  else digitalWrite(WHEEL_R, H);

}


