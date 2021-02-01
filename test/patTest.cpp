#include <Arduino.h>
#include <Bounce2.h>
#include "bcsjTimer.h"
#include <OneButton.h>
#include <Wire.h>






//---Instantiat Bounce objects for panel pushbuttons
Bounce debouncer1 = Bounce(); Bounce debouncer2 = Bounce(); 
Bounce debouncer3 = Bounce(); Bounce debouncer4 = Bounce();
Bounce debouncer5 = Bounce();
const byte debounceInterval = 5;


int routeChoice     = 0;
uint8_t pinRegister = 0;
int panelSelect     = 0;

uint8_t lastRoute   = 0;





//------------Setup turnout selection pins---
const uint8_t  routeA_pin {26},
               routeB_pin {27},
               routeC_pin {14},
               routeD_pin {12},
               routeE_pin {23};

const uint8_t LED_PIN {2};

 void blinkLED(int x)
 {
    for (int count {0}; count < x; ++count)
    {digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay (200);
    };
 }

void readPanel() 
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
    uint8_t newRoute = pinRegister;
    if ((lastRoute != newRoute) && 
        (pinRegister != 0)      && 
        (pinRegister == 1       ||
         pinRegister == 2       ||
         pinRegister == 4       ||
         pinRegister == 8       ||
         pinRegister == 16       )
       )
    {
      lastRoute = newRoute;
      routeChoice = newRoute;
      blinkLED(routeChoice);  //debug
    }
  }



void setup() 
{
  Serial.begin(115200);
  delay(1000);  //time to bring up serial monitor
  Wire.setClock(1000000L);


  //---setup the sensor pins
  pinMode(routeA_pin, INPUT_PULLUP);  pinMode(routeB_pin, INPUT_PULLUP);
  pinMode(routeC_pin, INPUT_PULLUP);  pinMode(routeD_pin, INPUT_PULLUP);
  pinMode(routeE_pin, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);

  //---setup the Bounce pins and intervals :
  debouncer1.attach(routeA_pin);   debouncer2.attach(routeB_pin);
  debouncer3.attach(routeC_pin);   debouncer4.attach(routeD_pin);
  debouncer5.attach(routeE_pin);

  //---setup interval in ms
  debouncer1.interval(debounceInterval);   
  debouncer2.interval(debounceInterval); 
  debouncer3.interval(debounceInterval);   
  debouncer4.interval(debounceInterval);
  debouncer5.interval(debounceInterval); 

//blinkLED(6);

}



void loop() {
  readPanel();
  panelSelect = pinRegister;
  Serial.print("pinRegister:  ");
  Serial.println(pinRegister);
  Serial.print("panelSelect:  ");
  Serial.println(pinRegister);
  Serial.print("              routeChoice:  ");
  Serial.println(routeChoice);

  //delay(1000);
  
 }