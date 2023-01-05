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
    PS4.begin("11:11:11:11:11:11");
    Serial.println("ready!");

  
}

void loop() 
{
    if(PS4.isConnected())
    {
        if(PS4.Triangle()) Serial.println("Trianlge!");
    }
    else
    {
         Serial.println("*");
    }


  
}