#include <Arduino.h>
#include <ArduinoBLE.h>
//#include <nrf.h>

#define WAKE_PIN P0_3   // D1
// Quadrature Encoder Pins (Hardware QDEC)
#define ENC_A_PIN P0_29  // P0.28 (D2)
#define ENC_B_PIN P0_28  // P0.29 (D3)
#define FlashAddress 0x000F0000

// BLE UUIDs matching the HTML page
#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define WRITE32_UUID "11111111-1234-5678-1234-56789abcdef0"
#define NOTIFY_UUID "22222222-1234-5678-1234-56789abcdef0"

// BLE objects
BLEService myService(SERVICE_UUID);
BLECharacteristic write32Char(WRITE32_UUID, BLEWrite, 5); // 5 bytes: id + 32-bit value
BLECharacteristic notifyChar(NOTIFY_UUID, BLENotify, 5);  // 5 bytes: id + 32-bit value

int32_t encoderCount = 0;
uint16_t colorWarm=0, colorCold=0;
uint32_t* flash = (uint32_t*)FlashAddress;

void wakeup(){} // Wird beim Aufwachen NICHT ausgeführt – aber muss existieren 

void sendNotify(uint8_t id, uint32_t val) {
  uint8_t response[5] = {id, (uint8_t)val, (uint8_t)(val >> 8), (uint8_t)(val >> 16), (uint8_t)(val >> 24)};
  notifyChar.writeValue(response, 5);
}

void setupBLE()
{
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Set advertised local name and service UUID
  BLE.setLocalName("Nachtlicht");  // Match HTML
  BLE.setAdvertisedService(myService);

  // Add characteristics to service
  myService.addCharacteristic(write32Char);
  myService.addCharacteristic(notifyChar);

  // Add service
  BLE.addService(myService);

  // Set initial values
  uint8_t initial[5] = {0, 0, 0, 0, 0};
  write32Char.writeValue(initial, 5);
  notifyChar.writeValue(initial, 5);

  // Start advertising
  BLE.advertise();

  Serial.println("BLE device active, waiting for connections...");
}

void shutDown()
{
  NRF_QDEC->TASKS_STOP = 1; 
  NRF_QDEC->ENABLE = 0; 
  NRF_PWM0->ENABLE = 0; 
  NRF_PWM1->ENABLE = 0; 
  NRF_UART0->ENABLE = 0; 
  NRF_TWIM0->ENABLE = 0; 
  NRF_SPIM0->ENABLE = 0;

  colorWarm += 1;
  colorCold *= 2;
  BLE.end();
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
  NRF_NVMC->ERASEPAGE = FlashAddress;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
  flash[0] = (colorWarm<<16) | colorCold;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
  flash[1] = encoderCount;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
  Serial.println(flash[0]);
  Serial.println(flash[1]);
  Serial.println("shutting down");
  
  NRF_POWER->SYSTEMOFF = 1;
}

void BLEhandler()
{
  if (write32Char.written()) 
  {
    const uint8_t* data = write32Char.value();
    uint8_t id = data[0];
    uint32_t val = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
    
    Serial.print("Received id: ");
    Serial.print(id);
    Serial.print(", value: ");
    Serial.println(val);

    if (id == 0) 
    {
      encoderCount = val;
    } else if (id == 1) 
    {
      colorCold = val;
    } else if (id == 2) 
    {
      colorWarm = val;
    } else if (id == 99) 
    {
      // Send current values
      sendNotify(0, encoderCount);
      sendNotify(1, colorCold);
      sendNotify(2, colorWarm);
      return; // Don't echo back for 99
    }

    // Echo back with same id and value
    sendNotify(id, val);
  }
}

void setup() {
  // Interne Pull-Ups für die Encoder-Pins aktivieren 
  pinMode(ENC_A_PIN, INPUT_PULLUP); 
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(WAKE_PIN, INPUT_PULLUP);
  pinMode(LED_BLUE, OUTPUT);

  Serial.begin(9600);
  uint8_t i=0;
  while(!Serial&&i++<10)
  {
    digitalWrite(LED_BLUE, LOW);
    delay(300);
    digitalWrite(LED_BLUE, HIGH);
    delay(300);
  }

  setupBLE();

  colorCold = flash[0] & 0xFFFF;
  colorWarm = (flash[0] >> 16) & 0xFFFF;
  encoderCount = flash[1];

  Serial.println("Warm: " + String(colorWarm) + " Cold: " + String(colorCold) + " Count: " + String(encoderCount));
  encoderCount = 0;
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
}

void loop() 
{
  static bool firstConnect = true;
  BLEDevice central = BLE.central();
  if(central&&firstConnect)
  {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    firstConnect = false;
  }
  if(!firstConnect)
  { 
    if(central.connected()) 
      BLEhandler();
    else
    {
      Serial.println("Disconnected from central");
      firstConnect = true;
    }
  }
 
  if(NRF_QDEC->EVENTS_REPORTRDY)
  {
    NRF_QDEC->TASKS_READCLRACC = 1; 
    encoderCount += NRF_QDEC->ACCREAD + NRF_QDEC->ACCDBLREAD;
    Serial.println(encoderCount);
    sendNotify(0, encoderCount);
    digitalWrite(LED_BLUE, LOW);
    NRF_QDEC->EVENTS_REPORTRDY = 0; // Clear event flag
  }
  if(abs(encoderCount) >= 600)
    shutDown();
}

