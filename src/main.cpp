/******************************************************************

                        LoLin32 psController 

                                                  қuran machr 2024
******************************************************************/

#include <Arduino.h>
#include <stdio.h>
#include <PS4Controller.h>

#define FASTLED_ALL_PINS_HARDWARE_SPI
#include <FastLED.h>

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>
#include "SPIFFS.h"

#include <Wire.h>

// -------  defines -----------------------------------------------

#define TRUE                            true
#define FALSE                           false
#define WAIT_ONE_SEC                    10000
#define WAIT_250_MSEC                   2500
#define WAIT_ONE_10MSEC                 100

#define H                               HIGH
#define L                               LOW

//  --- #define ON_BOARD_LED                    5

#define NUM_LEDS                        4
#define DATA_PIN                        23
#define CLOCK_PIN                       18

#define WHEEL_L                         2
#define WHEEL_R                         A4
#define WHEEL_L_DIRECTION               15 
#define WHEEL_R_DIRECTION               A5

#define BATTERY_LEVEL                   A3      // GPIO 39

#define REFV                            685.0     // factor

#define MFS                             0x1e
#define LEN                             21

#define EEPROM_SIZE                      100
#define EEPROM_ADDR                      0
#define EEPROM_SSID_ADDR                 0
#define EEPROM_PASSWORD_ADDR             40
#define EEPROM_MOTOR_SYS_ADDR            80

// -------  global Variables --------------------------------------

volatile int oneSecFlag;
volatile int qSecFlag;
volatile int tenMSecFlag;

hw_timer_t *timer = NULL;
void IRAM_ATTR myTimer(void);

volatile int vL, vR;

CRGB leds[NUM_LEDS];

volatile int pulse; 
volatile int connected;
volatile int LDir;
volatile int RDir;

volatile int pointer;
volatile float batteryLevel = 0.;
volatile int winkelL = 0; 
volatile int oldWinkelL = 0;
volatile int go;
volatile int startWiFi = 0;

String receivedText = ""; // Variable zum Speichern der empfangenen Daten
String ssidFromEEPROM = "";
String passwordFromEEPROM = "";
String motorSysFromEEPROM = "";
String receivedWord = "";

volatile int motorSys = 0;
                            
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

/*
IPAddress lclIP(192,168,2,219);
IPAddress gateway(192,168,2,1);
IPAddress subnet(255,255,255,0);
*/

const int led5 = 5;
bool ledState = 1;

// -------  functions ---------------------------------------------

void initSPIFFS()
{
    if (!SPIFFS.begin(true))
    {
        printf("An error has occurred while mounting SPIFFS\n");
    }
    else
    {
        printf("SPIFFS mounted successfully!\n");
    }
}

void initWiFi()
{
    char text[LEN];
    //WiFi.softAPConfig(lclIP, gateway, subnet);
    WiFi.mode(WIFI_STA);

    //WiFi.begin(ssid, password);
    WiFi.begin(ssidFromEEPROM, passwordFromEEPROM);

    printf("Connection to WiFi . . .");
    while ((WiFi.status() != WL_CONNECTED) && (startWiFi < 21))
    {
        delay(1000);
        printf(" .");
        startWiFi++;
    }

    //Serial.println("IP:");
    //Serial.println(WiFi.localIP());

    uint32_t ip = (uint32_t) WiFi.localIP();
    sprintf(text, "%u.%u.%u.%u", ip & 0xFF, (ip>>8) & 0xFF, (ip>>16) & 0xFF, (ip>>24) & 0xFF );
    printf("\nIP: %s\n", text);
}

String processor(const String& var)
{
    String battery;

    if (var == "STATE") 
    {
        if (digitalRead(led5))
        {
            ledState = 1; return "ON";
            printf("on\n");
        }
        else
        {
            ledState = 0; return "OFF";
            printf("off\n");
        }
    }

    if (var == "BATTERY")
    {
       battery = String(batteryLevel).c_str();
       return battery;
    }

    return String();
}

void notifyClients(String state)
{
    ws.textAll(state);
    //ws.text(1, state); // State
}


void handleWebSocketMessage(void *arg, uint8_t * data, size_t len)
{
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len && info->opcode == WS_TEXT)
    {
        data[len] = 0;
        if (strcmp((char*)data, "bON") == 0)
        {
            ledState = 1;
            notifyClients("ON");

            printf("handleWebSocketMessage: on\n");

        }
        if (strcmp((char*)data, "bOFF") == 0)
        {
            ledState = 0;
            notifyClients("OFF");
            printf("handleWebSocketMessage: off\n");
        }
    }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient * client, AwsEventType type, 
             void * arg, uint8_t * data, size_t len)
{
    switch(type)
    {
        case WS_EVT_CONNECT: 
             printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        break;

        case WS_EVT_DISCONNECT:
             printf("WebSocket client #%u disconnected\n", client->id());
        break;

        case WS_EVT_DATA:
             handleWebSocketMessage(arg, data, len);
        break;

        case WS_EVT_PONG:
        case WS_EVT_ERROR:
        break;
    }
}

void initWebSocket()
{
    ws.onEvent(onEvent);
    server.addHandler(&ws);
}


void store2EEPROM(String word, int address);

void store2EEPROM(String word, int address)
{
    int i;

    for (i = 0; i < word.length(); i++)
    {
        EEPROM.write(address + i, word[i]);
    }
    EEPROM.write(address + word.length(), '\0');    
    EEPROM.commit();  // Änderungen speichern
}

String readFromEEPROM(int address);
String readFromEEPROM(int address)
{
  String word = "";
  char c;
  while ((c = EEPROM.read(address++)) != '\0') 
  { // Lies Zeichen bis zur Null-Terminierung
    word += c;
  }
  return word;
}



void setup() 
{
    uint8_t dataToWrite = 42; 
    uint8_t dataToRead;
    char text[LEN];

    Serial.begin(115200);
    printf("start!");
    go = 1; 

    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("EEPROM initialisieren fehlgeschlagen!");
        return;
    }

    //EEPROM.write(EEPROM_ADDR, dataToWrite);
    //EEPROM.commit();  // Änderungen speichern
    //dataToRead = 0;
    //dataToRead = EEPROM.read(EEPROM_ADDR);
    
    Serial.println("Daten gelesen: \n" + String(dataToRead));

    ssidFromEEPROM = readFromEEPROM(EEPROM_SSID_ADDR);
    printf("im EEPROM gefunden: .%s.\n", ssidFromEEPROM);
    passwordFromEEPROM = readFromEEPROM(EEPROM_PASSWORD_ADDR);
    printf("im PASSWOR gefunden: .%s.\n", passwordFromEEPROM);
    motorSysFromEEPROM = readFromEEPROM(EEPROM_MOTOR_SYS_ADDR);
    motorSys = (char)motorSysFromEEPROM[0] - '0';
    printf("MotorSystem: .%d.\n", motorSys);


    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &myTimer, true);
    timerAlarmWrite(timer, 100, true);  // 0.1 msec
    timerAlarmEnable(timer);
    
    oneSecFlag = FALSE; 
    qSecFlag = FALSE;
    tenMSecFlag = FALSE; 
    PS4.begin("10:20:30:40:50:62");


    printf("ready!\n");

    // -- pinMode(ON_BOARD_LED, OUTPUT);
    pinMode(led5, OUTPUT);

    pinMode(WHEEL_L, OUTPUT);
    pinMode(WHEEL_R, OUTPUT);
    pinMode(WHEEL_L_DIRECTION, OUTPUT);
    pinMode(WHEEL_R_DIRECTION, OUTPUT);
    
    pinMode(BATTERY_LEVEL, INPUT);

    // -- digitalWrite(ON_BOARD_LED, L); // on ! ... blue 

    digitalWrite(led5, L);


    digitalWrite(WHEEL_L_DIRECTION, L);
    digitalWrite(WHEEL_R_DIRECTION, H);

    digitalWrite(WHEEL_L, L); // stop !
    digitalWrite(WHEEL_R, L); // stop !

    pulse = 0;
    connected = 0;
    LDir = 0;
    RDir = 1;
    pointer = 0;

    vL = vR = 0;

// Fast Leds:

    FastLED.addLeds<SK9822, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
    
    leds[0] = CRGB{255, 255, 255}; // R B G
    leds[1] = CRGB{255, 255, 255};
    leds[2] = CRGB{255, 255, 255};
    leds[3] = CRGB{255, 255, 255};

    FastLED.show();

//24    // Magnetfeldsensor:  preparation phase ...
//24    Wire.begin();
//24    Wire.beginTransmission(MFS);
//24    Wire.write(0x02);        // select mode register
//24    Wire.write(0x00);        // continuous measurement mode
//24    Wire.endTransmission();


    initSPIFFS();
    initWiFi();
    initWebSocket();


    server.on("/", HTTP_GET,
    [](AsyncWebServerRequest * request)
    {
        request->send(SPIFFS, "/index.html", "text/html", false, processor);
    });

    server.serveStatic("/", SPIFFS, "/");

    server.on("/logo", HTTP_GET, 
    [](AsyncWebServerRequest *request)
    {
        request->send(SPIFFS, "/logo.png", "image/png");
    }
             );

    // Start server:

    server.begin();
}

void loop() 
{
    uint8_t leftstickXDATA0 = 0;
    uint8_t leftstickXDATA1 = 0;
    static int t1 = 0;
    String battery;
    int akn;
    uint8_t xH, xL, yH, yL, zH, zL;
    uint16_t x, y, z;
    static uint16_t xmax = 0, xmin = 0xffff, ymax = 0, ymin = 0xffff, zmax = 0, zmin = 0xffff;
    uint16_t xSpan, xMidd, ySpan, yMidd, zSpan, zMidd;
    float r, m, n;
    int angleXY, angleYZ, angleZX;
    String winkel;
    int startIndex, endIndex;

    ws.cleanupClients();

    if (ledState == 1) {digitalWrite(led5, 0); go = 1;} else { digitalWrite(led5, 1); go = 0;}

    if (oneSecFlag == TRUE)  // all Seconds 
    {
            oneSecFlag = FALSE;

            batteryLevel = analogRead(BATTERY_LEVEL) / REFV;

            //batteryLevel += 3.75;  // for tests only
            
            //if (batteryLevel > 359) batteryLevel = 0;

            battery = String(batteryLevel).c_str();

            battery = "BAT" + battery;

            notifyClients(battery);

    }


    if (go == 0)
    {
        vL = vR = 0;

        leds[0] = CRGB{0, 0, 0}; // R B G
        leds[1] = CRGB{0, 0, 0};
        leds[2] = CRGB{0, 0, 0};
        leds[3] = CRGB{0, 0, 0};

        FastLED.show();
    }
    else
    {

        if (qSecFlag)  // all 250 msec ... 
        {
            qSecFlag = FALSE;

            pulse = pulse ? 0 : 1;

            // digitalWrite(ON_BOARD_LED, pulse);
            // Serial.print(x); Serial.println(" V");

            if(PS4.isConnected())
            {
                connected = 1;

                // digitalWrite(ON_BOARD_LED, pulse);  // off

                leds[0] = CRGB{0, 0, 255}; // R B G
                leds[1] = CRGB{0, 255, 0};
                leds[2] = CRGB{255, 0, 255};  // Yellow: FF00FF
                leds[3] = CRGB{255, 0, 0};

                FastLED.show();

                if(PS4.Triangle()) printf("Triangle!\n"); 
                if(PS4.Up()) printf("Up!\n");
            
            }
            else
            {
                pointer++; if (pointer >= 4) pointer = 0;

                switch(pointer)
                {
                    case 0:
                        leds[0] = CRGB{255, 255, 255}; // R B G
                        leds[1] = CRGB{255, 255, 255};
                        leds[2] = CRGB{0, 0, 0};  // Yellow: FF00FF
                        leds[3] = CRGB{0, 0, 0};
                    break;

                    case 1:
                        leds[0] = CRGB{0, 0, 0}; // R B G
                        leds[1] = CRGB{255, 255, 255};
                        leds[2] = CRGB{255, 255, 255};  // Yellow: FF00FF
                        leds[3] = CRGB{0, 0, 0};
                    break;

                    case 2:
                        leds[0] = CRGB{0, 0, 0}; // R B G
                        leds[1] = CRGB{0, 0, 0};
                        leds[2] = CRGB{255, 255, 255};  // Yellow: FF00FF
                        leds[3] = CRGB{255, 255, 255};
                    break;

                    case 3:
                        leds[0] = CRGB{255, 255, 255}; // R B G
                        leds[1] = CRGB{0, 0, 0};
                        leds[2] = CRGB{0, 0, 0};  // Yellow: FF00FF
                        leds[3] = CRGB{255, 255, 255};
                    break;
                }

                FastLED.show();


                while (Serial.available() > 0) {
                char receivedChar = Serial.read(); // Einzelnes Zeichen lesen

                // Wenn ein Zeilenumbruch empfangen wird, Ausgabe und Text zurücksetzen
                if (receivedChar == '\n') {
                Serial.println("Empfangene Daten: " + receivedText); // Daten ausgeben
                   receivedText = ""; // Textfeld zurücksetzen
                } else {
                receivedText += receivedChar; // Zeichen an das Textfeld anhängen

                receivedText.trim();
//#JASON Format: 
                startIndex = 0; 
                startIndex = receivedText.indexOf("\"ssid\":\"");
                //"ssid":"   das sind 8 Zeichen!
                if (startIndex != -1)
                {
                    startIndex += 8;
                    endIndex = receivedText.indexOf("\"", startIndex);
                    if (endIndex != -1)
                    {
                        receivedWord = receivedText.substring(startIndex, endIndex);
                        printf("\nUserWort erkannt .%s.", receivedWord);
                        store2EEPROM(receivedWord, EEPROM_SSID_ADDR);
                    }
                }

                startIndex = receivedText.indexOf("\"password\":\"");
                //"password":"   das sind 12 Zeichen!
                if (startIndex != -1)
                {
                    startIndex += 12;
                    endIndex = receivedText.indexOf("\"", startIndex);
                    if (endIndex != -1)
                    {
                        receivedWord = receivedText.substring(startIndex, endIndex);
                        printf("\nPassword erkannt .%s.", receivedWord);
                        store2EEPROM(receivedWord, EEPROM_PASSWORD_ADDR);
                    }
                }


                startIndex = receivedText.indexOf("\"motor-system\":\"");
                //"motor-system":"   das sind 15 Zeichen!
                if (startIndex != -1)
                {
                    startIndex += 16; // wegen dem '-' Zeichen ? 
                    endIndex = receivedText.indexOf("\"", startIndex);
                    if (endIndex != -1)
                    {
                        receivedWord = receivedText.substring(startIndex, endIndex);
                        printf("\nMotorsystem erkannt .%s.", receivedWord);
                        store2EEPROM(receivedWord, EEPROM_MOTOR_SYS_ADDR);
                    }
                }



                if (receivedText == "RESET") 
                {
                    Serial.println("Rebooting...");
                    ESP.restart(); // Neustart des ESP32
                }
        }
    }



            }

//24            Wire.beginTransmission(MFS);
//24            Wire.write(0x03);
//24            akn = Wire.endTransmission();

//24            Wire.requestFrom(MFS, 6);
//24            if (6 <= Wire.available())
//24            {
//24                xH = Wire.read(); xL = Wire.read();
//24                yH = Wire.read(); yL = Wire.read();
//24                zH = Wire.read(); zL = Wire.read();

//24                x = ((xH + 128) << 8) + xL;  // + 128 to position -> avoid negativ values
//24                y = ((yH + 128) << 8) + yL;
//24                z = ((zH + 128) << 8) + zL;
//24            }

//24            if (x > xmax) xmax = x; if (x < xmin) xmin = x; 
//24            if (y > ymax) ymax = y; if (y < ymin) ymin = y; 
//24            if (z > zmax) zmax = z; if (z < zmin) zmin = z; 

//24            xSpan = (xmax - xmin) / 2; xMidd = (xmax + xmin) / 2; r = (float)(x - xMidd) / xSpan; 
//24            ySpan = (ymax - ymin) / 2; yMidd = (ymax + ymin) / 2; m = (float)(y - yMidd) / ySpan; 
//24            zSpan = (zmax - zmin) / 2; zMidd = (zmax + zmin) / 2; n = (float)(z - zMidd) / zSpan; 

//24            angleXY = (int)(atan2(r, m) * 180 / M_PI);
//24            angleYZ = (int)(atan2(m, n) * 180 / M_PI);
//24            angleZX = (int)(atan2(n, r) * 180 / M_PI);


//24            winkelL = - angleXY;

//            printf("winkel xy %d yz %d zx %d \n", angleXY, angleYZ, angleZX);



            if (LDir == H) winkelL = winkelL + vL/10.; else winkelL = winkelL - vL/10.;
            if (RDir == H) winkelL = winkelL + vR/10.; else winkelL = winkelL - vR/10.;

            if (abs((winkelL - oldWinkelL)) > 5) // to avoid interferece
            {
                winkel = String(winkelL).c_str();
                winkel = "WIN" + winkel;
                notifyClients(winkel);
                oldWinkelL = winkelL;
            }
        }    

        if (tenMSecFlag)
        {
            // "entprellt" 

            tenMSecFlag = FALSE;

            if (connected)
            {
                vL = PS4.L2Value();
                vR = PS4.R2Value();
            
                if(PS4.L1()) {LDir = LDir ? 0 : 1;  digitalWrite(WHEEL_L_DIRECTION, LDir);}
                if(PS4.R1()) {RDir = RDir ? 0 : 1;  digitalWrite(WHEEL_R_DIRECTION, RDir);}
            }
        }
    }

}    

//** Timer Interrupt:   ******************************************

void IRAM_ATTR myTimer(void)   // periodic timer interrupt, expires each 0.1 msec
{
    static int32_t otick  = 0;
    static int32_t qtick = 0;
    static int32_t mtick = 0;
    static unsigned char ramp = 0;
    
    otick++;
    qtick++;
    mtick++;
    ramp++;

    if (otick >= WAIT_ONE_SEC) 
    {
        oneSecFlag = TRUE;
        otick = 0; 
    }

    if (qtick >= WAIT_250_MSEC) 
    {
        qSecFlag = TRUE;
        qtick = 0; 
    }

    if (mtick >= WAIT_ONE_10MSEC) 
    {
        tenMSecFlag = TRUE;
        mtick = 0; 
    }

    // PWM:
    if (motorSys == 0) 
    {
        if (ramp > vL) digitalWrite(WHEEL_L, L);  else digitalWrite(WHEEL_L, H);
        if (ramp > vR) digitalWrite(WHEEL_R, L);  else digitalWrite(WHEEL_R, H);
    }
    if (motorSys == 1)
    { 
        if (LDir) if (ramp < vL) digitalWrite(WHEEL_L, L);  else digitalWrite(WHEEL_L, H);
        else      if (ramp > vL) digitalWrite(WHEEL_L, L);  else digitalWrite(WHEEL_L, H);

        if (RDir) if (ramp < vR) digitalWrite(WHEEL_R, L);  else digitalWrite(WHEEL_R, H);
        else      if (ramp > vR) digitalWrite(WHEEL_R, L);  else digitalWrite(WHEEL_R, H);

    }

}
