// BatteryLogger_nRF52.cpp
// Seeed XIAO nRF52840 (Arduino) – Langzeit-Logger für Akkuspannung
// BLE: ArduinoBLE. Messung exakt wie in VBat.cpp (AR_INTERNAL2V4, 12 Bit, AT_40_US, Median + Mapping 7155).

#include <Arduino.h>
#include <ArduinoBLE.h>          // BLE-Stack (wie in deinem Projekt)
#include <MedianFilterLib.h>     // identisch zu deinem VBat.cpp

// -------------------------------------------------------------
// ADC / Messmethode exakt wie in VBat.cpp
// -------------------------------------------------------------
#define VBAT_PIN P0_31
MedianFilter<int> gMedian(50);
MedianFilter<int> RefMedian(50);
volatile uint16_t gMedianNow = 0;  // Median (ADC-Rohwert)
volatile uint16_t RefMedianNow = 0;

static void adcInit() {
  pinMode(P0_31, INPUT);
  pinMode(P0_2, INPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(P0_14, OUTPUT);
  pinMode(P0_13, OUTPUT);
  digitalWrite(P0_14, LOW);
  digitalWrite(P0_13, LOW);

  analogReference(AR_INTERNAL2V4);
  analogReadResolution(12);
  analogAcquisitionTime(AT_40_US);

  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
}

// Map (Median-ADC) -> Millivolt exakt wie in VBat.cpp: 0..4093 -> 0..7155 mV (fine-tuned)
static inline uint16_t adcMedianToMilliVolt(uint16_t medianAdc) {
  long mv = map((long)medianAdc, 0L, 4095L, 0L, /*9771L*/7106L);
  if (mv < 0) mv = 0;
  if (mv > 65535) mv = 65535;
  return (uint16_t)mv;
}

// Hintergrundabtastung (alle 250 ms) wie in VBat.cpp
static const uint32_t ADC_TICK_MS = 250;
static uint32_t s_lastAdcTick = 0;
//static const uint16_t RefNominal = 1881;
static void adcBackgroundTick() {
  uint32_t now = millis();
  if (now - s_lastAdcTick >= ADC_TICK_MS) {
    s_lastAdcTick = now;
    uint16_t raw = analogRead(VBAT_PIN);
    //uint16_t rawRef = analogRead(P0_2);
    gMedianNow = (uint16_t)gMedian.AddValue((int)raw);
    //RefMedianNow = (uint16_t)RefMedian.AddValue((int)rawRef);
    //gMedianNow = gMedianNow * (RefMedianNow/RefNominal);
  }
}

// -------------------------------------------------------------
// Logger-Daten
// -------------------------------------------------------------
struct __attribute__((packed)) Sample {
  uint32_t t_s;   // Sekunden seit Start
  uint16_t mV;    // Millivolt
  uint16_t adc;   // ADC (Median-Rohwert)
};

static const size_t   MAX_SAMPLES    = 4096;       // ≈32 KB (8 B * 4096)
static Sample         s_ring[MAX_SAMPLES];
static uint32_t       s_totalSamples = 0;          // monotone Gesamtanzahl
static size_t         s_count        = 0;          // aktuell im Ring
static size_t         s_head         = 0;          // nächste Schreibposition
static bool           s_logging      = true;
static uint32_t       s_period_s     = 60;         // Intervall (Sekunden)

static void pushSample(const Sample& s) {
  s_ring[s_head] = s;
  s_head = (s_head + 1) % MAX_SAMPLES;
  if (s_count < MAX_SAMPLES) s_count++;
  s_totalSamples++;
}
static inline uint32_t uptime_s() { return millis() / 1000; }

// -------------------------------------------------------------
// BLE UUIDs / Characteristics
// -------------------------------------------------------------
#define SVC_UUID     "b0b00001-6e2b-4fcb-9da9-5a0b1d2f00a1"
#define CHR_SAMPLE   "b0b00002-6e2b-4fcb-9da9-5a0b1d2f00a1" // Notify: 8B = [u32 t_s][u16 mV][u16 adc]
#define CHR_VOLT     "b0b00003-6e2b-4fcb-9da9-5a0b1d2f00a1" // Read/Notify: 2B = mV
#define CHR_CTRL     "b0b00004-6e2b-4fcb-9da9-5a0b1d2f00a1" // Write: Kommandos
#define CHR_INFO     "b0b00005-6e2b-4fcb-9da9-5a0b1d2f00a1" // Read: 16B = [u32 total][u32 period][u32 max][u32 count]
#define CHR_ADC      "b0b00006-6e2b-4fcb-9da9-5a0b1d2f00a1" // Read/Notify: 2B = adc (raw)

BLEService        gSvc(SVC_UUID);
BLECharacteristic gChrSample (CHR_SAMPLE,  BLENotify,            8);
BLECharacteristic gChrVolt   (CHR_VOLT,    BLERead | BLENotify,  2);
BLECharacteristic gChrCtrl   (CHR_CTRL,    BLEWrite,            16);
BLECharacteristic gChrInfo   (CHR_INFO,    BLERead,             16);
BLECharacteristic gChrAdc    (CHR_ADC,     BLERead | BLENotify,  2);

// Streaming-State (für CTRL=0x20)
static bool     s_streamActive = false;
static uint32_t s_streamRem    = 0;
static uint32_t s_streamIdx    = 0;   // absoluter Index
static uint32_t s_streamStep   = 1;

// -------------------------------------------------------------
// LE-Helfer (statt kaputter Lambdas)
// -------------------------------------------------------------
static inline uint32_t rdU32LE(const uint8_t* data, size_t len, int off) {
  if ((size_t)(off + 4) > len) return 0;
  return  (uint32_t)data[off]
        | ((uint32_t)data[off+1] << 8)
        | ((uint32_t)data[off+2] << 16)
        | ((uint32_t)data[off+3] << 24);
}
static inline void wrU32LE(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

// optional: nicht genutzt, kann entfernt werden
static void sendVoltageNotify(uint16_t mV) {
  uint8_t b[2] = { (uint8_t)(mV & 0xFF), (uint8_t)(mV >> 8) };
  gChrVolt.writeValue(b, 2);
}

// Absoluten Index -> Ring-Index
static bool getByAbsIndex(uint32_t absIdx, Sample& out) {
  if (s_count == 0) return false;
  uint32_t oldest = (s_totalSamples >= s_count) ? (s_totalSamples - s_count) : 0;
  if (absIdx < oldest || absIdx >= s_totalSamples) return false;
  uint32_t rel = absIdx - oldest;                          // 0..s_count-1
  size_t pos = ((s_head + MAX_SAMPLES) - s_count + rel) % MAX_SAMPLES;
  out = s_ring[pos];
  return true;
}

// CTRL-Kommandos
static void handleCtrlWrite(const uint8_t* data, size_t len) {
  if (len < 1) return;
  uint8_t cmd = data[0];

  switch (cmd) {
    case 0x01: s_logging = true;  break;   // Start
    case 0x00: s_logging = false; break;   // Stop
    case 0x02:                      // Clear buffer
      s_head = s_count = 0;
      s_totalSamples = 0;
      break;

    case 0x10: { // Set period (seconds)
      if (len >= 1+4) {
        uint32_t p = rdU32LE(data, len, 1);
        if (p == 0) p = 1;
        s_period_s = p;
      }
    } break;

    case 0x20: { // Streaming anfordern: startIndex, step, count
      if (len >= 1+12) {
        uint32_t start = rdU32LE(data, len, 1);
        uint32_t step  = rdU32LE(data, len, 5);
        uint32_t cnt   = rdU32LE(data, len, 9);
        if (step == 0) step = 1;

        uint32_t oldest = (s_totalSamples >= s_count) ? (s_totalSamples - s_count) : 0;
        if (start < oldest) start = oldest;
        if (start >= s_totalSamples) { s_streamActive = false; break; }

        s_streamIdx  = start;
        s_streamStep = step;
        s_streamRem  = cnt;
        s_streamActive = (s_streamRem > 0);
      }
    } break;

    default: break;
  }
}

// Pro Loop ein paar Pakete (Throttle), um BLE-Queue nicht zu fluten
static void streamTick() {
  if (!s_streamActive) return;
  BLEDevice central = BLE.central();
  if (!central || !central.connected()) { s_streamActive = false; return; }

  uint8_t pkt[8];
  int sentThisRound = 0;
  const int MAX_PER_TICK = 8;

  while (s_streamActive && sentThisRound < MAX_PER_TICK) {
    Sample s;
    if (!getByAbsIndex(s_streamIdx, s)) {
      s_streamActive = false;
      break;
    }
    // Packen: [u32 t_s][u16 mV][u16 adc] little-endian
    pkt[0] = (uint8_t)(s.t_s & 0xFF);
    pkt[1] = (uint8_t)((s.t_s >> 8) & 0xFF);
    pkt[2] = (uint8_t)((s.t_s >> 16) & 0xFF);
    pkt[3] = (uint8_t)((s.t_s >> 24) & 0xFF);
    pkt[4] = (uint8_t)(s.mV & 0xFF);
    pkt[5] = (uint8_t)((s.mV >> 8) & 0xFF);
    pkt[6] = (uint8_t)(s.adc & 0xFF);
    pkt[7] = (uint8_t)((s.adc >> 8) & 0xFF);
    gChrSample.writeValue(pkt, sizeof(pkt));

    if (s_streamRem > 0) s_streamRem--;
    if (s_streamRem == 0) { s_streamActive = false; break; }
    if (s_streamIdx >= 0xFFFFFFF0UL) { s_streamActive = false; break; }
    s_streamIdx += s_streamStep;
    sentThisRound++;
  }
}

// -------------------------------------------------------------
// Setup / Loop
// -------------------------------------------------------------
void setup() {
  adcInit();  // ADC wie in VBat.cpp

  if (!BLE.begin()) {
    while (1) delay(1000);
  }

  BLE.setLocalName("BattLogger_nRF52");
  BLE.setAdvertisedService(gSvc);

  gSvc.addCharacteristic(gChrSample);
  gSvc.addCharacteristic(gChrVolt);
  gSvc.addCharacteristic(gChrCtrl);
  gSvc.addCharacteristic(gChrInfo);
  gSvc.addCharacteristic(gChrAdc);

  BLE.addService(gSvc);

  // initiale Werte
  { uint8_t v2[2] = {0,0}; gChrVolt.writeValue(v2, 2); }
  { uint8_t a2[2] = {0,0}; gChrAdc.writeValue(a2, 2); }
  {
    uint8_t info[16];
    wrU32LE(&info[0],  0);
    wrU32LE(&info[4],  s_period_s);
    wrU32LE(&info[8],  MAX_SAMPLES);
    wrU32LE(&info[12], 0);
    gChrInfo.writeValue(info, sizeof(info));
  }

  BLE.advertise();

  // Median “anwärmen”
  //for (int i=0; i<50; ++i) { (void)gMedian.AddValue((int)analogRead(VBAT_PIN)); delay(2); }
}

static uint32_t s_lastLogTick = 0;

void loop() {
  // BLE-Service
  BLEDevice central = BLE.central();
  if (central) {
    if (gChrCtrl.written()) {
      int len = gChrCtrl.valueLength();
      uint8_t buf[20]; if (len > 20) len = 20;
      gChrCtrl.readValue(buf, len);
      handleCtrlWrite(buf, len);
    }
  }

  // ADC-Hintergrundabtastung
  adcBackgroundTick();

  // Letzten Einzelwerte bereitstellen (Read/Notify-Quelle)
  uint16_t mV = adcMedianToMilliVolt(gMedianNow);
  { uint8_t vb[2] = { (uint8_t)(mV & 0xFF), (uint8_t)(mV >> 8) }; gChrVolt.writeValue(vb, 2); }
  { uint8_t ab[2] = { (uint8_t)(gMedianNow & 0xFF), (uint8_t)(gMedianNow >> 8) }; gChrAdc.writeValue(ab, 2); }

  // Periodisches Logging
  uint32_t now_ms = millis();
  if (s_logging && (now_ms - s_lastLogTick >= s_period_s * 1000UL)) {
    s_lastLogTick = now_ms;

    Sample s{ uptime_s(), mV, gMedianNow };
    pushSample(s);

    // INFO aktualisieren: [u32 total][u32 period][u32 max][u32 count]
    uint8_t info[16];
    wrU32LE(&info[0],  s_totalSamples);
    wrU32LE(&info[4],  s_period_s);
    wrU32LE(&info[8],  MAX_SAMPLES);
    wrU32LE(&info[12], s_count);
    gChrInfo.writeValue(info, sizeof(info));

    // Optional: letzten Wert auch via VOLT/ADC notifyen
    // sendVoltageNotify(mV);
    // (für ADC gäbe es analog einen sendAdcNotify, falls gewünscht)
  }

  // Streaming (auf Anforderung 0x20)
  streamTick();

  delay(2);
}

