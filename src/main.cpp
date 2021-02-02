#include <Arduino.h>
#include <Bounce2.h>
#include "bcsjTimer.h"
//#include <OneButton.h>
//#include <Wire.h>

const byte MAX_LADDER_TRACKS {17};
const byte MAX_TURNOUTS      {17};


//---Turnout bit masks are encoded for shift register input. T0 is
//   always lsb for shift register 
const uint16_t	THROWN_S1	  {0x0001};	//0000000000000001	1
const uint16_t	THROWN_S2	  {0x0002};	//0000000000000010	2
const uint16_t	THROWN_S3	  {0x0004};	//0000000000000100	4
const uint16_t	THROWN_S4	  {0x0008};	//0000000000001000	8
const uint16_t	THROWN_S5	  {0x0010};	//0000000000010000	16
const uint16_t	THROWN_S6	  {0x0020};	//0000000000100000	32
const uint16_t	THROWN_S7	  {0x0040};	//0000000001000000	64
const uint16_t	THROWN_S8   {0x0080};	//0000000010000000	128
const uint16_t	THROWN_S9	  {0x0100};	//0000000100000000	256
const uint16_t	THROWN_S10  {0x0200};	//0000001000000000	512
const uint16_t	THROWN_S11	{0x0400};	//0000010000000000	1024
const uint16_t	THROWN_S12	{0x0800};	//0000100000000000	2048
const uint16_t	THROWN_S13	{0x1000};	//0001000000000000	4096
const uint16_t	THROWN_S14	{0x2000};	//0010000000000000	8192
const uint16_t	THROWN_S15	{0x4000};	//0100000000000000	16384
const uint16_t	THROWN_S16	{0x8000};	//1000000000000000	32768

struct turnoutMap {
  uint8_t       numTracks;
  uint8_t       startTrack;
  uint8_t       defaultTrack;
  bool          revL;
  char          mapName[16];
  uint16_t      routes[MAX_LADDER_TRACKS];
};

/**********************************************************************
*    Channel assignment for turnouts and LED's for Patterson Creek
*       THROWN_S(n) indicates thrown position for a turnout channel
*       THROWN_S(n) indicates a lighted LED for a LED channel                              
***********************************************************************

TURNOUT 1         THROWN_S1, 
TURNOUT 2         THROWN_S2, 
TURNOUT 3         THROWN_S3, 
TURNOUT 4         THROWN_S4,  
TURNOUT 5         THROWN_S5, 
LED_6             THROWN_S6,  
LED_7             THROWN_S7,
LED_8             THROWN_S8,
LED_9             THROWN_S9,
LED_10            THROWN_S10,
LED_11            THROWN_S11, 
LED_12            THROWN_S12,
LED_13            THROWN_S13,
LED_14            THROWN_S14,
NOT USED          THROWN_S15,
NOT USED          THROWN_S16,

***********************************************************************/


/**********************************************************************
*    Test Yard IS USED TO TEST SINGLE TORTOISES.  
*    A SINGLE TORTOISE IS SELECTED FOR EACH ROUTE.                         
**********************************************************************/
turnoutMap PattersonCreek = {         // Map #0
             16,               // numTracks
             0,                // startTrack
             1,                // default track
             false,            // have reverse track?
             "Patterson Creek",           // map name
/* trk W0          */  0, 
/* ROUTE A         */  THROWN_S6+THROWN_S7+THROWN_S8+THROWN_S9+THROWN_S11, 
/* ROUTE B         */  THROWN_S2+THROWN_S6+THROWN_S13+THROWN_S14, 
/* ROUTE C         */  THROWN_S1+THROWN_S3+THROWN_S8+THROWN_S9+THROWN_S10,
/* ROUTE D         */  THROWN_S6+THROWN_S7+THROWN_S8+THROWN_S9+THROWN_S11,  
/* ROUTE E         */  THROWN_S4+THROWN_S5+THROWN_S9+THROWN_S12+THROWN_S14 

};


turnoutMap test = {         // Map #1
             16,               // numTracks
             1,                // startTrack
             0,                // default track
             false,            // have reverse track?
             "Test",           // map name
/* trk W0          */  0, 
/* SWITCH A        */  THROWN_S1, 
/* SWITCH B        */  THROWN_S2, 
/* SWITCH C        */  THROWN_S3, 
/* SWITCH D        */  THROWN_S4,  
/* SWITCH E        */  THROWN_S5, 
/* LED_MAIN2EAST   */  THROWN_S6, 
/* LED_MAIN1EAST   */  THROWN_S7,
/* LED_MAINWEST    */  THROWN_S8,
/* LED_CUMBERLAND  */  THROWN_S9,
/* LED_SWITCH A    */  THROWN_S10,
/* LED_SWITCH B    */  THROWN_S11, 
/* LED_SWITCH C    */  THROWN_S12,
/* LED_SWITCH D    */  THROWN_S13,
/* LED_SWITCH E    */  THROWN_S14,
/* NOT USED        */  THROWN_S15,
/* NOT USED        */  THROWN_S16,
};




//--------------Map yard memory addresses with pointer for crntMap----
const turnoutMap *mapData[2] = {
  &PattersonCreek,   // #0
  &test              // #1
};



//-----Setup pins for 74HC595 shift register 
const int latchPin = 33;   
const int clockPin = 32;   
const int dataPin  = 25; 

//-----declare latch function----
void writeTrackBits( uint16_t track);

//---Instantiate a bcsjTimer.h object for screen sleep
bcsjTimer  timerTortoise;
unsigned long interval_Tortoise     = 1000000L * 3;         //Tortoise run time interval

// Instantiate a Bounce object
Bounce debouncer1 = Bounce(); Bounce debouncer2 = Bounce(); 
Bounce debouncer3 = Bounce(); Bounce debouncer4 = Bounce();
Bounce debouncer5 = Bounce();

int     routeActive  = 0;
uint8_t pinRegister  = 0;
int     panelSelect  = 0;
uint8_t crntMap      = 0;

uint8_t lastRoute   = 0;
bool newChoice = false;

//---------------SETUP STATE Machine and State Functions-------------------
enum {HOUSEKEEP, STAND_BY, TRACK_SETUP} mode;
void runHOUSEKEEP();
void runSTAND_BY();
void runTRACK_SETUP();

//------------ Function:  read route switches on fascia -------------------
void readPanel();  
//------------ Function: write route data to shift registerr ---------------
//void writeTrackBits( uint16_t track);

void blinkLED(int x);
void blinkLEDdash(int x);
const uint8_t LED_PIN {2};


//------------Setup turnout selection pins-----
const byte routeA_pin {26},
           routeB_pin {27},
           routeC_pin {14},
           routeD_pin {12},
           routeE_pin {23};



void setup() 
{
  Serial.begin(115200);
  delay(1000);  //time to bring up serial monitor
  
  pinMode(LED_PIN, OUTPUT);

  //---Setup the sensor pins
  pinMode(routeA_pin, INPUT_PULLUP); pinMode(routeB_pin, INPUT_PULLUP);
  pinMode(routeC_pin, INPUT_PULLUP); pinMode(routeD_pin, INPUT_PULLUP);
  pinMode(routeE_pin, INPUT_PULLUP);

  //---Shift register pins
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin,  OUTPUT);  
  pinMode(clockPin, OUTPUT);

  //---setup the Bounce pins and intervals :
  debouncer1.attach(routeA_pin); debouncer2.attach(routeB_pin);
  debouncer3.attach(routeC_pin);  debouncer4.attach(routeD_pin);
  debouncer5.attach(routeE_pin);

  debouncer1.interval(5);           debouncer2.interval(5); // interval in ms
  debouncer3.interval(5);           debouncer4.interval(5);
  debouncer5.interval(5);

  writeTrackBits(mapData[crntMap]->routes[mapData[crntMap]->defaultTrack]);

  blinkLEDdash(1) ;
  blinkLED(3);
  delay(700);
  blinkLED(1);
  blinkLEDdash(1);
  blinkLED(3);
  delay(700);
  blinkLED(3);
  delay(1000);


}



void loop() {
                          Serial.println("void loop:      ");
                          
 
 if (mode == HOUSEKEEP)        {runHOUSEKEEP();}
 else if (mode == STAND_BY)    {runSTAND_BY();}
 else if (mode == TRACK_SETUP) {runTRACK_SETUP();}
 
}


//---this function is included.for possible future use---
void runHOUSEKEEP()
  {
                          Serial.println("           RUNHOUSKEEP:  ");
    runSTAND_BY();
  }


void runSTAND_BY()
  {
                          Serial.println("           STANDBY:  ");
    
    readPanel();
  }

void runTRACK_SETUP()
  {
                          Serial.println("           TRACKSETUP:  ");
    writeTrackBits(mapData[crntMap]->routes[routeActive]);
    newChoice = false;
    runHOUSEKEEP();
  }

void readPanel() 
  {
    
                          Serial.println("           readPANEL:  ");
    do
    {
    //---check panel switches---
    debouncer1.update();
    debouncer2.update();
    debouncer3.update();
    debouncer4.update();
    debouncer5.update();

    //---update pinRegister--- 
    if (debouncer1.read() == LOW) bitSet(pinRegister, 0);
      else bitClear(pinRegister,0);
    if (debouncer2.read() == LOW) bitSet(pinRegister, 1);
      else bitClear(pinRegister,1);
    if (debouncer3.read() == LOW) bitSet(pinRegister, 2);
      else bitClear(pinRegister,2);
    if (debouncer4.read() == LOW) bitSet(pinRegister, 3);
      else bitClear(pinRegister,3);
    if (debouncer5.read() == LOW) bitSet(pinRegister, 4);
      else bitClear(pinRegister,4);

    /*---evaluate: if change; for zero; and if a valid number (not: *
    *    two buttons held down)                                     *
    *---------------------------------------------------------------*/
    uint8_t testRoute = 0;

    switch (pinRegister)
    {
    case 1:
      testRoute = 1;
      break;
    case 2:
      testRoute = 2;
      break;
    case 4:
      testRoute = 3;
      break;
    case 8:
      testRoute = 4;
      break;
    case 16:
      testRoute = 5;
      break;
    
    default:
      break;
    }

    uint8_t newRoute = testRoute;

    if ((lastRoute != newRoute) && 
        (pinRegister != 0)      //&& 
        //(pinRegister == 1       ||
         //pinRegister == 2       ||
         //pinRegister == 4       ||
         //pinRegister == 8       ||
         //pinRegister == 16       )
       )
      {
        lastRoute = newRoute;
        routeActive = newRoute;
        newChoice = true;

        blinkLED(routeActive);  //debug

        runTRACK_SETUP();
      }
    }
    while (newChoice == false);
}  

void writeTrackBits(uint16_t track)
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, (track >> 8));
  shiftOut(dataPin, clockPin, MSBFIRST, track);
  digitalWrite(latchPin, HIGH);
            //*/--DEBUG: 
            Serial.print("trackFunction: ");
            Serial.println(track);
            Serial.println(track, BIN);  
}  

void blinkLED(int x)
 {
    for (int count {0}; count < x; ++count)
    {digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay (100);
    };
 }
void blinkLEDdash(int x)
 {
    for (int count {0}; count < x; ++count)
    {digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay (300);
    };
 }