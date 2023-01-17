/******************************************************************
                        LoLin32 psController
                                                    қuran jan 2023
******************************************************************/
#include <Arduino.h>
#include <PS4Controller.h>
//#include <WiFi.h>

void setup() 
{
    Serial.begin(115200);
    Serial.println("start!");
//    PS4.begin("90:cd:b6:e9:bb:50");
    PS4.begin("10:20:30:40:50:62");
    Serial.println("ready!");
}
void loop() 
{
    static int t1 = 0;
    if ( millis() - t1 > 1000)
    {
        t1 = millis();   
        if(PS4.isConnected())
        {
            Serial.println("connected!");
            if(PS4.Triangle()) Serial.println("Triangle!");
        }
        else
        {
             Serial.print("*");
        }
    }    
}    
