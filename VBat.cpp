#include <Arduino.h>
#include <MedianFilterLib.h>
//#include "nrf.h"

 
MedianFilter<int> median(100);

void setup()
{
  Serial.begin(115200);
  while(!Serial);
  delay(2000);
  Serial.println("ADC test");
  pinMode(LED_BLUE, OUTPUT);
  pinMode(P0_14, OUTPUT);
  pinMode(P0_13, OUTPUT);
  pinMode(P0_31, INPUT);
  digitalWrite(P0_14, LOW);
  digitalWrite(P0_13, LOW);
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
  if(i<100)
  {
    Serial.print(i);
    Serial.print("\t");
    i++;
  }
  ASum = median.AddValue(AValue);
  float VBat = float(map(ASum, 0, 4093, 0, 7155))/1000.0; //theoretischer Wert 7106 (1000*VRef*(R1+R2)/R2), hier wurde fine-getuned
  nrf_gpio_pin_toggle(LED_BLUE);
  Serial.println(VBat);
  
  delay(250);
}

