#include <Arduino.h>
#include <nrf.h>

#define WAKE_PIN P0_3   // D1
// Quadrature Encoder Pins (Hardware QDEC)
#define ENC_A_PIN P0_28  // P0.28 (D2)
#define ENC_B_PIN P0_29  // P0.29 (D3)

long encoderCount = 0;


void wakeup() { // Wird beim Aufwachen NICHT ausgeführt – aber muss existieren 
  // Nur um zu zeigen, dass der Interrupt funktioniert
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Interne Pull-Ups für die Encoder-Pins aktivieren 
  pinMode(ENC_A_PIN, INPUT_PULLUP); 
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(WAKE_PIN, INPUT_PULLUP);
  pinMode(LED_BLUE, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(WAKE_PIN), wakeup, FALLING);
  
  // Initialize QDEC peripheral
  NRF_QDEC->ENABLE = 1;  // Enable QDEC
  
  // Set pin assignments
  NRF_QDEC->PSEL.A = ENC_A_PIN;
  NRF_QDEC->PSEL.B = ENC_B_PIN;
  NRF_QDEC->PSEL.LED = 0xFFFFFFFF;  // No LED pin used
  
  // Configure QDEC
  NRF_QDEC->DBFEN = 1;  // Enable debounce filter
  NRF_QDEC->SAMPLEPER = QDEC_SAMPLEPER_SAMPLEPER_128us;  // Sample period
  NRF_QDEC->REPORTPER = QDEC_REPORTPER_REPORTPER_10Smpl;  // Report period
  
  // Start QDEC
  NRF_QDEC->TASKS_START = 1;
  digitalWrite(LED_BLUE, LOW); // Indicate setup complete
}

void loop() {
  // put your main code here, to run repeatedly:
  // Read the accumulator value
  encoderCount = NRF_QDEC->ACC;
  Serial.println(encoderCount);
  if(abs(encoderCount) >= 100) {
    NRF_QDEC->TASKS_STOP = 1; 
    NRF_QDEC->ENABLE = 0; 
    NRF_PWM0->ENABLE = 0; 
    NRF_PWM1->ENABLE = 0; 
    NRF_UART0->ENABLE = 0; 
    NRF_TWIM0->ENABLE = 0; 
    NRF_SPIM0->ENABLE = 0;
    //BLE.end();
    NRF_POWER->SYSTEMOFF = 1;
  }
  delay(100);
}


