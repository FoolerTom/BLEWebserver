#include <Arduino.h>
#include <ArduinoBLE.h>

// BLE UUIDs matching the HTML page
#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define WRITE32_UUID "11111111-1234-5678-1234-56789abcdef0"
#define NOTIFY_UUID "22222222-1234-5678-1234-56789abcdef0"

// BLE objects
BLEService myService(SERVICE_UUID);
BLECharacteristic write32Char(WRITE32_UUID, BLEWrite, 5); // 5 bytes: id + 32-bit value
BLECharacteristic notifyChar(NOTIFY_UUID, BLENotify, 5);  // 5 bytes: id + 32-bit value

// Current values
uint32_t brightness = 0;
uint32_t timer1_seconds = 0;
uint32_t timer2_seconds = 0;

void sendNotify(uint8_t id, uint32_t val) {
  uint8_t response[5] = {id, (uint8_t)val, (uint8_t)(val >> 8), (uint8_t)(val >> 16), (uint8_t)(val >> 24)};
  notifyChar.writeValue(response, 5);
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

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

void loop() {
  // Listen for BLE peripherals to connect
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      // Check if write32Char has been written
      if (write32Char.written()) {
        const uint8_t* data = write32Char.value();
        uint8_t id = data[0];
        uint32_t val = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
        
        Serial.print("Received id: ");
        Serial.print(id);
        Serial.print(", value: ");
        Serial.println(val);

        if (id == 0) {
          timer1_seconds = val;
        } else if (id == 1) {
          timer2_seconds = val;
        } else if (id == 2) {
          brightness = val;
        } else if (id == 99) {
          // Send current values
          sendNotify(0, timer1_seconds);
          sendNotify(1, timer2_seconds);
          sendNotify(2, brightness);
          continue; // Don't echo back for 99
        }

        // Echo back with same id and value
        sendNotify(id, val);
      }
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}