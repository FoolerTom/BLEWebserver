//Funktionstüchtig, Timeränderung zu shutdown() gefixt und softStart verändert. Noch nicht tiefer getestet
//+Akkumanagement müsste nochmals überprüft werden

#include <Arduino.h>
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"
#include <bluefruit.h>
#include <MedianFilterLib.h>

#define PWMperiod       32000

#define WARM_PIN        11      // unchanged – warm LED output
#define WARMER_PIN      12      // unchanged – warmer LED output
#define VBAT_PIN        31      // analogue battery sense
#define BUTTON_PIN      3         // formerly P0_3

#define ENC_A_PIN       29         // formerly P0_29
#define ENC_B_PIN       28         // formerly P0_28
#define BAT_EN_PIN      14     // enable voltage divider for battery measurement
#define BAT_DISCON_PIN  13     // disconnect battery (pulls low to disconnect)
#define CHARGE_PIN      13     // control charging current
#define CHARGE_INDI_PIN 17

#define IN_UP           12
#define IN_DOWN         4
#define INP             0
#define DISCON          2
#define OUTP            3
#define EN_INTERRUPT    (3<<16)

const uint32_t FLASH_DATA_ADDR = 0x001000;

const uint32_t SampleNumber = 20;

const bool completely = true;

// BLE UUIDs matching the HTML page
#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define WRITE32_UUID "11111111-1234-5678-1234-56789abcdef0"
#define NOTIFY_UUID "22222222-1234-5678-1234-56789abcdef0"

//----------------------------------------------------------------------------------------
// --- WebLog (optional) -------------------------------------------------------
#define WEBLOG_ENABLE 1  // <- auf 0 setzen oder Zeile entfernen, um alles zu deaktivieren

#if WEBLOG_ENABLE
  // eigener Notify-Characteristic für Logtexte
  #define LOG_UUID "33333333-1234-5678-1234-56789abcdef0"
  BLECharacteristic logChar(LOG_UUID); // 200B Payload (abhängig von MTU)
#endif
//----------------------------------------------------------------------------------------

SPIFlash_Device_t const p25q16h{
  .total_size = (1UL << 21),  // 2MiB
  .start_up_time_us = 10000,
  .manufacturer_id = 0x85,
  .memory_type = 0x60,
  .capacity = 0x15,
  .max_clock_speed_mhz = 55,
  .quad_enable_bit_mask = 0x02,
  .has_sector_protection = 1,
  .supports_fast_read = 1,
  .supports_qspi = 1,
  .supports_qspi_writes = 1,
  .write_status_register_split = 1,
  .single_status_byte = 0,
  .is_fram = 0,
};

Adafruit_FlashTransport_QSPI QflashTransport/*(PIN_QSPI_CS, SPI_2)*/;      // CS for QSPI Flash
Adafruit_SPIFlash Qflash(&QflashTransport);
// BLE objects

BLEService serviceMain(SERVICE_UUID);
BLECharacteristic write32Char(WRITE32_UUID);
BLECharacteristic notifyChar(NOTIFY_UUID);

MedianFilter<uint32_t> ADCMedian(SampleNumber);

int32_t brightness=0, lightColor=0, batteryLevel=0;
int32_t timerOn=0, timerDim=0;
uint32_t startUpTime = 0, ms = 0;
uint16_t pwm_seq[4] = {PWMperiod,PWMperiod,0,0};
int16_t saadc_buffer=0;
volatile bool tick100ms = false;

//---------------------------------------------------------------------------------------------------------------
#if WEBLOG_ENABLE
static void weblog_write(const uint8_t* data, size_t len) {
  // BLE Notify in Chunks (typisch 20..200B; wir nehmen 180 als konservativ)
  const size_t CHUNK = 180;
  size_t off = 0;
  while (off < len) {
    size_t n = (len - off) < CHUNK ? (len - off) : CHUNK;
    logChar.notify(data + off, n);
    off += n;
    // kurze Pause optional, wenn du Flooding vermeidest
    // delay(1);
  }
}

// printf-ähnlich, formatiert in lokalen Buffer (Stack)
static void weblog_printf(const char* fmt, ...) {
  char buf[256]; // kurze Logzeilen
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  weblog_write((const uint8_t*)buf, strlen(buf));
}

// println-Variante
static void weblog_println(const char* s) {
  weblog_write((const uint8_t*)s, strlen(s));
  const char nl = '\n';
  weblog_write((const uint8_t*)&nl, 1);
}

// bequemer Makro wie Serial.print/println:
  #define WEBLOG(...)       weblog_printf(__VA_ARGS__)
  #define WEBLOGLN(s)       weblog_println(s)
#else
  // wenn deaktiviert: rausoptimieren
  #define WEBLOG(...)
  #define WEBLOGLN(s)
#endif
//---------------------------------------------------------------------------------------------------------------

extern "C" void SAADC_IRQHandler(void)
{
  NRF_SAADC->EVENTS_RESULTDONE = 0;
  ADCMedian.AddValue(saadc_buffer);

  NRF_SAADC->TASKS_START = 1;
}

extern "C" void TIMER1_IRQHandler(void)
{
    if (NRF_TIMER1->EVENTS_COMPARE[0])
    {
        NRF_TIMER1->EVENTS_COMPARE[0] = 0;
        tick100ms = true;
    }
}

void QflashSleep (bool turnOff = true)
{  
  if(turnOff)
  {
    //Qflash.end();
    QflashTransport.begin();
    QflashTransport.runCommand(0xB9);
    QflashTransport.end();
    
  }
  else
  {
    QflashTransport.begin();
    QflashTransport.runCommand(0xAB);
    Qflash.begin(&p25q16h, 1);
  }
}

void saveToFlash() 
{
  int32_t vars[4] = {timerOn, timerDim, brightness, lightColor};
  uint8_t data[16];
  memcpy(data, vars, sizeof(vars));

  QflashSleep(false);

  Qflash.eraseSector(FLASH_DATA_ADDR>>12);
  Qflash.writeBuffer(FLASH_DATA_ADDR, data, sizeof(data));

  QflashSleep();
}

void loadFromFlash() 
{
  uint8_t data[16];
  int32_t vars[4];

  QflashSleep(false);

  Qflash.readBuffer(FLASH_DATA_ADDR, data, sizeof(data));

  memcpy(vars, data, sizeof(vars));

  timerOn = constrain(vars[0], 0, 86399);
  timerDim = constrain(vars[1], 0, 86399);
  brightness = constrain(vars[2], 1, 100);
  lightColor = constrain(vars[3], 0, 100);

  QflashSleep();
}

void shutDown(bool batteryDisconnect = false)
{  
    saveToFlash();

    NRF_P0->PIN_CNF[ENC_A_PIN] = DISCON;
    NRF_P0->PIN_CNF[ENC_B_PIN] = DISCON;
    NRF_P0->PIN_CNF[VBAT_PIN] = DISCON;
    NRF_P0->PIN_CNF[CHARGE_INDI_PIN] = DISCON;
    NRF_P0->PIN_CNF[BAT_EN_PIN] = DISCON;

    NRF_P0->PIN_CNF[BUTTON_PIN] |= EN_INTERRUPT;

    while(pwm_seq[0]>pwm_seq[1])
    {
        pwm_seq[1]++;
        NRF_PWM0->TASKS_SEQSTART[0] = 1;
        delayMicroseconds(25);
    }
    while(pwm_seq[0]<pwm_seq[1])
    {
        pwm_seq[0]++;
        NRF_PWM0->TASKS_SEQSTART[0] = 1;
        delayMicroseconds(25);
    }
    while(pwm_seq[0]<PWMperiod)
    {
        pwm_seq[0]++;
        pwm_seq[1]++;
        NRF_PWM0->TASKS_SEQSTART[0] = 1;
        delayMicroseconds(50);
    }

    if(batteryDisconnect)
        NRF_P1->PIN_CNF[BAT_DISCON_PIN] = IN_DOWN;
    NRF_POWER->SYSTEMOFF = 1;
}

void setupPWM()
{
    NRF_PWM0->PSEL.OUT[0] = WARMER_PIN|32;
    NRF_PWM0->PSEL.OUT[1] = WARM_PIN|32;         
    NRF_PWM0->PSEL.OUT[2] = 0xFFFFFFFF;   // OUT2 nicht genutzt
    NRF_PWM0->PSEL.OUT[3] = 0xFFFFFFFF;   // OUT3 nicht genutzt


    NRF_PWM0->ENABLE      = 1;
    NRF_PWM0->MODE        = PWM_MODE_UPDOWN_Up;                     // Edge-aligned
    NRF_PWM0->PRESCALER   = PWM_PRESCALER_PRESCALER_DIV_2;          // 1 = 16 MHz
    NRF_PWM0->COUNTERTOP  = PWMperiod;                              // 16000 = 1 kHz
    
    NRF_PWM0->DECODER =
        (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |
        (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);

    NRF_PWM0->SEQ[0].PTR      = (uint32_t)pwm_seq;
    NRF_PWM0->SEQ[0].CNT      = 4;       // Pflicht: 4 Werte für OUT0..OUT3
    NRF_PWM0->SEQ[0].REFRESH  = 0;       // jede Periode Werte neu lesen
    NRF_PWM0->SEQ[0].ENDDELAY = 0;

    
    NRF_PWM0->LOOP   = 0xFFFF;                      // maximale Wiederholzahl
    NRF_PWM0->SHORTS = PWM_SHORTS_LOOPSDONE_SEQSTART0_Msk;

    startUpTime = millis();

    uint16_t targetWarmer = PWMperiod - (uint32_t)(PWMperiod/100) * brightness * lightColor / 100;
    uint16_t targetWarm = PWMperiod - (uint32_t)(PWMperiod/100) * brightness * (100 - lightColor) / 100;

    while(pwm_seq[0]>targetWarmer && pwm_seq[1]>targetWarm)
    {
      pwm_seq[0]--;
      pwm_seq[1]--;
      NRF_PWM0->TASKS_SEQSTART[0] = 1;
      delayMicroseconds(50);
    }
    while(pwm_seq[0]>targetWarmer)
    {
      pwm_seq[0]--;
      NRF_PWM0->TASKS_SEQSTART[0] = 1;
      delayMicroseconds(25);
    }
    while(pwm_seq[1]>targetWarm)
    {
      pwm_seq[1]--;
      NRF_PWM0->TASKS_SEQSTART[0] = 1;
      delayMicroseconds(25);
    }
}

void sendNotify(uint8_t id, uint32_t val) 
{
    
uint8_t response[5] =
    {
      id,
      (uint8_t)val,
      (uint8_t)(val >> 8),
      (uint8_t)(val >> 16),
      (uint8_t)(val >> 24)
    };
    notifyChar.notify(response, 5);

}

void readBat()
{
  static uint8_t shutDownCounter = 0;
  int32_t BatBuffer = ADCMedian.GetFiltered();
  int32_t BatPcnt = map(BatBuffer, 1800L, 2360L, 0L, 100L);
  if (BatPcnt < 0)
  {
    BatPcnt = 0;
    shutDownCounter++;
    if(shutDownCounter>=3)
      shutDown(completely);
  }
  else
    shutDownCounter = 0;
  if (BatPcnt > 100) BatPcnt = 100;
  if(!(NRF_P0->IN & (1<<CHARGE_INDI_PIN)))
      BatPcnt |= 1<<31;
  batteryLevel = BatPcnt;
  sendNotify(4, batteryLevel);    
  WEBLOG("\nADC raw: %u %u", batteryLevel, BatBuffer);
}

void write32_callback(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
    if (len != 5) return;

    uint8_t id = data[0];
    int32_t val = data[1] | (data[2]<<8) | (data[3]<<16) | (data[4]<<24);

    //WEBLOG("RX id=%u val=%lu\n", (unsigned)id, (unsigned long)val);

    switch (id)
    {
      case 0:
        if(timerDim+val>=60)
            timerOn = val;
        else
            val = timerOn = 60-timerDim;
        ms = startUpTime = millis();
        break;
      case 1: 
        if(timerOn+val>=60)
            timerDim = val;
        else
            val = timerDim = 60-timerOn;
        ms = startUpTime = millis();
        break;
      case 2: brightness = val = constrain(val, 1, 100); break;
      case 3: lightColor = val = constrain(val, 0, 100); break;

      case 99:
        sendNotify(0, timerOn);
        delay(50);
        sendNotify(1, timerDim);
        delay(50);
        sendNotify(2, brightness);
        delay(50);
        sendNotify(3, lightColor);
        delay(50);
        readBat();
        return;
    }

    // Echo back
    sendNotify(id, val);
}

void setupBLE()
{
  Bluefruit.autoConnLed(false);

  Bluefruit.begin();
  //Bluefruit.setTxPower(4);
  Bluefruit.setName("Nachtlicht");

  serviceMain.begin();

  write32Char.setProperties(CHR_PROPS_WRITE);
  write32Char.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  write32Char.setWriteCallback(write32_callback);
  write32Char.begin();

  notifyChar.setProperties(CHR_PROPS_NOTIFY);
  notifyChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  notifyChar.begin();

#if WEBLOG_ENABLE
  logChar.setProperties(CHR_PROPS_NOTIFY);
  logChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  logChar.begin();
  uint8_t zero = 0;
  logChar.notify(&zero, 1);
#endif

  // ------------------------------------------------------
  // BLE Advertising
  // ------------------------------------------------------

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  
  Bluefruit.Advertising.addService(serviceMain);
  Bluefruit.Advertising.addName();
  
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(60);
}

void setupBatMan() 
{
  NRF_P0->PIN_CNF[VBAT_PIN] = INP;
  NRF_P0->PIN_CNF[CHARGE_INDI_PIN] = INP;
  NRF_P0->PIN_CNF[BAT_EN_PIN] = OUTP;
  NRF_P0->PIN_CNF[CHARGE_PIN] = OUTP;
  NRF_P1->PIN_CNF[BAT_DISCON_PIN] = IN_UP/*|DISCON*/;

  NRF_P0->OUTCLR = (1<<BAT_EN_PIN)|(1<<CHARGE_PIN);

  NRF_SAADC->ENABLE = 0;
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;
  NRF_SAADC->CH[0].CONFIG = 0x00050200;
  NRF_SAADC->CH[0].PSELP = 8; //AIN7 = P0.31 = VBAT_PIN
  NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;
  NRF_SAADC->RESULT.PTR = (uint32_t)&saadc_buffer;
  NRF_SAADC->RESULT.MAXCNT = 1;
  
  NRF_SAADC->EVENTS_END = 0;
  NRF_SAADC->EVENTS_RESULTDONE = 0;
  NRF_SAADC->INTENSET = (1<<3);
  NRF_SAADC->ENABLE = 1;

  NRF_SAADC->TASKS_START = 1;
  NRF_SAADC->TASKS_SAMPLE = 1;
}

void setupTimerPpi()
{
    NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER1->PRESCALER = 4; // 1 MHz (16MHz/2^4)

    NRF_TIMER1->CC[0] = 100000; // 100 ms

    NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;

    // TIMER -> SAADC
    NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TIMER1->EVENTS_COMPARE[0];
    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_SAADC->TASKS_SAMPLE;
    NRF_PPI->CHENSET = PPI_CHENSET_CH0_Msk;

    // Interrupt zusätzlich aktivieren
    NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
    NVIC_EnableIRQ(TIMER1_IRQn);

    NRF_TIMER1->TASKS_START = 1;

    NVIC_EnableIRQ(SAADC_IRQn);
}

void setupEncoder()
{
  NRF_P0->PIN_CNF[ENC_A_PIN] = IN_UP;
  NRF_P0->PIN_CNF[ENC_B_PIN] = IN_UP;
  NRF_P0->PIN_CNF[BUTTON_PIN] = IN_UP;

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

void setup() 
{
    loadFromFlash();
    setupPWM();
    setupBatMan();
    setupEncoder();
    setupBLE();
    setupTimerPpi();
}

void loop() 
{
    ms = millis();
    static bool fallingEdge = true, noShutDown = false;
    static uint32_t fallTime = 0, noShutDownTime = 0, batReadTicks = 0;
    bool ButtonPressed = !(NRF_P0->IN&(1<<BUTTON_PIN));
    
    if(tick100ms)
    {
        tick100ms = false;
        if(ButtonPressed&&fallingEdge)
        {
            fallTime = ms;
            fallingEdge = false;
            if(ms-noShutDownTime>500)
                noShutDown = false;
        }
        if(!fallingEdge&&!ButtonPressed)
        {
            if(ms-fallTime>100)
            {
                if(noShutDown)
                    fallingEdge = true;
                else
                    shutDown();
            }
        }
        if(NRF_QDEC->EVENTS_REPORTRDY)
        {
            if(abs(NRF_QDEC->ACC)>=2)
            {
                NRF_QDEC->TASKS_READCLRACC = 1;
                if(ButtonPressed)
                {
                    lightColor += NRF_QDEC->ACCREAD/2 /*+ NRF_QDEC->ACCDBLREAD*/;
                    lightColor = constrain(lightColor, 0, 100);
                    sendNotify(3, lightColor);
                    noShutDown = true;
                    noShutDownTime = ms;
                }
                else
                {
                    brightness += NRF_QDEC->ACCREAD/2 /*+ NRF_QDEC->ACCDBLREAD*/;
                    brightness = constrain(brightness, 1, 100);
                    sendNotify(2, brightness);
                }
            }
            NRF_QDEC->EVENTS_REPORTRDY = 0; // Clear event flag
        }
        if(!((++batReadTicks)%50))
        {
            readBat();
            batReadTicks = 0;
            WEBLOG("\nPWM: %u \t %u", pwm_seq[0], pwm_seq[1]);
            WEBLOG("\nStart Up: %u %u", startUpTime, millis());
            WEBLOG("\nTimer: %u %u", timerOn, timerDim);
        }

        if((ms-startUpTime) < ((uint32_t)timerOn*1000)) //preset brightness for timerOn seconds
        {
            pwm_seq[0] = PWMperiod - (uint32_t)(PWMperiod/100) * brightness * lightColor / 100;
            pwm_seq[1] = PWMperiod - (uint32_t)(PWMperiod/100) * brightness * (100 - lightColor) / 100;
            NRF_PWM0->TASKS_SEQSTART[0] = 1;
        }
        else if((ms-startUpTime) < ((uint32_t)(timerOn+timerDim)*1000)) //dimming down for timerDim seconds
        {
            uint32_t dimFactor = 1000 - (uint32_t)1000 * (ms - startUpTime - timerOn*1000) / (timerDim*1000);
            pwm_seq[0] = PWMperiod - (uint32_t)(PWMperiod/100) * brightness * lightColor * dimFactor / 100000;
            pwm_seq[1] = PWMperiod - (uint32_t)(PWMperiod/100) * brightness * (100 - lightColor) * dimFactor / 100000;
            NRF_PWM0->TASKS_SEQSTART[0] = 1;
        }
        else
            shutDown();
    }
    __WFI();
}
