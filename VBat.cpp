#include <Arduino.h>
//#include "nrf.h"

 


void setup()
{
  Serial.begin(115200);
  while(!Serial);
  delay(2000);
  Serial.println("ADC test");
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(P0_14, OUTPUT);
  pinMode(P0_13, OUTPUT);
  pinMode(P0_31, INPUT);//VBat lesen an diesem Pin
  digitalWrite(P0_14, LOW);//VBat lesen ermöglichen, wenn dieser Pin = LOW.
  digitalWrite(P0_13, LOW);//P0_13 LOW=hoher Ladestrom (100mA)
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  analogReference(AR_INTERNAL2V4);
  analogReadResolution(12);
  analogAcquisitionTime(AT_40_US);  //genauere Werte mit hoher Acquisition Time. 40us ist max.
}

void loop()
{
  uint16_t AValue = analogRead(P0_31);
  static uint16_t ASum = 0;
  static uint8_t i=0;
  if(++i<=10)
    ASum += AValue;
  else
  {
    ASum/=10;
    float VBat = float(map(ASum, 0, 4093, 0, 7140))/100.0; //theoretischer Wert 7106, hier wurde fine-getuned
    nrf_gpio_pin_toggle(LED_BLUE);
    Serial.println(VBat);
    ASum = 0;
    i=0;
  } 
  
  delay(50);
}

