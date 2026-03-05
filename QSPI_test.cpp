#include <Arduino.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_FlashTransport.h>

// QSPI Pins für SEEED XIAO BLE nRF52840
#define QSPI_SCK_PIN  21
#define QSPI_CS_PIN   25
#define QSPI_IO0_PIN  20
#define QSPI_IO1_PIN  24
#define QSPI_IO2_PIN  22
#define QSPI_IO3_PIN  23

// QSPI Transport für nRF52
Adafruit_FlashTransport_QSPI flashTransport(QSPI_SCK_PIN, QSPI_CS_PIN, QSPI_IO0_PIN, QSPI_IO1_PIN, QSPI_IO2_PIN, QSPI_IO3_PIN);

// SPIFlash Objekt
Adafruit_SPIFlash flash(&flashTransport);

// Buffer für Tests
#define BUFFER_SIZE 1024
uint8_t buffer[BUFFER_SIZE];

void printBuffer(uint32_t start, uint32_t count) {
  Serial.print("Daten ab Adresse ");
  Serial.print(start);
  Serial.print(": ");
  for (uint32_t i = 0; i < count; i++) {
    if (i > 0) Serial.print(" ");
    Serial.print("0x");
    if (buffer[i] < 16) Serial.print("0");
    Serial.print(buffer[i], HEX);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }

  Serial.println("SPIFlash Test für XIAO nRF52840");

  // SPI initialisieren
  SPI.begin();

  // SPIFlash initialisieren
  if (!flash.begin()) {
    Serial.println("Fehler: SPIFlash konnte nicht initialisiert werden!");
    while (1);
  }

  Serial.print("Flash-Größe: ");
  Serial.print(flash.size() / 1024);
  Serial.println(" KB");

  Serial.println("Flash-Info:");
  Serial.print("  JEDEC ID: 0x");
  Serial.println(flash.getJEDECID(), HEX);
  uint8_t jedec_id[3];
  flashTransport.readCommand(SFLASH_CMD_READ_JEDEC_ID, jedec_id, 3);
  Serial.print("  Hersteller: 0x");
  Serial.println(jedec_id[0], HEX);

  // Test 1: Aktuelle Daten lesen
  Serial.println("\n--- Test 1: Daten lesen ---");
  uint32_t startTime = millis();
  flash.readBuffer(0, buffer, BUFFER_SIZE);
  uint32_t endTime = millis();
  Serial.print("Lesezeit: ");
  Serial.print(endTime - startTime);
  Serial.println(" ms");
  printBuffer(0, 16);  // Erste 16 Bytes anzeigen

  // Test 2: 4KB Sektor löschen
  Serial.println("\n--- Test 2: 4KB Sektor löschen ---");
  startTime = millis();
  flash.eraseSector(0);
  endTime = millis();
  Serial.print("Löschzeit: ");
  Serial.print(endTime - startTime);
  Serial.println(" ms");

  // Test 3: Daten schreiben
  Serial.println("\n--- Test 3: Daten schreiben ---");
  // Buffer mit Testdaten füllen
  for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
    buffer[i] = i % 256;
  }
  startTime = millis();
  flash.writeBuffer(0, buffer, BUFFER_SIZE);
  endTime = millis();
  Serial.print("Schreibzeit: ");
  Serial.print(endTime - startTime);
  Serial.println(" ms");

  // Test 4: Geschriebene Daten lesen und verifizieren
  Serial.println("\n--- Test 4: Daten verifizieren ---");
  uint8_t verifyBuffer[BUFFER_SIZE];
  startTime = millis();
  flash.readBuffer(0, verifyBuffer, BUFFER_SIZE);
  endTime = millis();
  Serial.print("Lesezeit: ");
  Serial.print(endTime - startTime);
  Serial.println(" ms");

  bool dataOK = true;
  for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
    if (verifyBuffer[i] != (i % 256)) {
      dataOK = false;
      break;
    }
  }

  if (dataOK) {
    Serial.println("Daten erfolgreich verifiziert!");
  } else {
    Serial.println("Daten-Fehler bei Verifikation!");
  }

  printBuffer(0, 16);  // Erste 16 Bytes anzeigen

  Serial.println("\nSPIFlash Test abgeschlossen!");
}
void loop()
{

}
