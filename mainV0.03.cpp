#include <Arduino.h>
#include <ArduinoBLE.h>
#include <MedianFilterLib.h>

//----------------------------------------------------------------------------------------
#include <stdarg.h>   // <- Wichtig für va_list, va_start, va_end
#include <stdio.h>    // optional: für vsnprintf()
//----------------------------------------------------------------------------------------



#define WARM_PIN P1_12
#define WARMER_PIN P1_11
#define VBAT_PIN P0_31
#define BUTTON_PIN P0_3   // D1
// Quadrature Encoder Pins (Hardware QDEC)
#define ENC_A_PIN P0_29  // P0.28 (D2)
#define ENC_B_PIN P0_28  // P0.29 (D3)
#define FlashAddress 0x000F0000

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
  BLECharacteristic logChar(LOG_UUID, BLENotify, 200); // 200B Payload (abhängig von MTU)
#endif
//----------------------------------------------------------------------------------------


// BLE objects
BLEService myService(SERVICE_UUID);
BLECharacteristic write32Char(WRITE32_UUID, BLEWrite, 5); // 5 bytes: id + 32-bit value
BLECharacteristic notifyChar(NOTIFY_UUID, BLENotify, 5);  // 5 bytes: id + 32-bit value

uint32_t brightness=0, lightColor=0, batteryLevel=0;
uint32_t timerOn=0, timerDim=0;
uint32_t* flash = (uint32_t*)FlashAddress;
uint16_t pwm_seq[4] = {4000,12000,0,0};

MedianFilter<int> ADCMedian(50);


const bool completely = true;

//---------------------------------------------------------------------------------------------------------------
#if WEBLOG_ENABLE
static void weblog_write(const uint8_t* data, size_t len) {
  // BLE Notify in Chunks (typisch 20..200B; wir nehmen 180 als konservativ)
  const size_t CHUNK = 180;
  size_t off = 0;
  while (off < len) {
    size_t n = (len - off) < CHUNK ? (len - off) : CHUNK;
    logChar.writeValue(data + off, n);
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
  #define WEBLOG(...)       do{}while(0)
  #define WEBLOGLN(s)       do{}while(0)
#endif
//---------------------------------------------------------------------------------------------------------------

void wakeup(){}

void sendNotify(uint8_t id, uint32_t val) 
{
    uint8_t response[5] = {id, (uint8_t)val, (uint8_t)(val >> 8), (uint8_t)(val >> 16), (uint8_t)(val >> 24)};
    notifyChar.writeValue(response, 5);
}

void setupBLE()
{
    // Initialize BLE
    while(!BLE.begin()); 

    // Set advertised local name and service UUID
    BLE.setLocalName("Nachtlicht");  // Match HTML
    BLE.setAdvertisedService(myService);

    // Add characteristics to service
    myService.addCharacteristic(write32Char);
    myService.addCharacteristic(notifyChar);

    //---------------------------------------------------------
    #if WEBLOG_ENABLE
    myService.addCharacteristic(logChar);
    // initialer leerer Wert (einige Stacks mögen das)
    uint8_t z = 0;
    logChar.writeValue(&z, 1);
    #endif
    //---------------------------------------------------------

    // Add service
    BLE.addService(myService);

    // Set initial values
    uint8_t initial[5] = {0, 0, 0, 0, 0};
    write32Char.writeValue(initial, 5);
    notifyChar.writeValue(initial, 5);

    // Start advertising
    BLE.advertise();

    //---------------------------------------------------------
    WEBLOGLN("BLE bereit, Werbung gestartet.");
    //---------------------------------------------------------
}

void shutDown(bool batteryDisconnect = false)
{
    NRF_QDEC->TASKS_STOP = 1; 
    NRF_QDEC->ENABLE = 0; 
    NRF_PWM0->ENABLE = 0; 
    NRF_PWM1->ENABLE = 0; 
    NRF_UART0->ENABLE = 0; 
    NRF_TWIM0->ENABLE = 0; 
    NRF_SPIM0->ENABLE = 0;

    pinMode(P1_11, INPUT_PULLUP);
    pinMode(P1_12, INPUT_PULLUP);

    BLE.end();
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    NRF_NVMC->ERASEPAGE = FlashAddress;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    flash[0] = timerOn;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    flash[1] = timerDim;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    flash[2] = brightness;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    flash[3] = lightColor;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    delay(100);

    if(batteryDisconnect)   //the battery gets disconnected. Wakeup only through USB charching
        pinMode(P1_13, INPUT_PULLDOWN);
    NRF_POWER->SYSTEMOFF = 1;
}

void setupPWM()
{
    NRF_PWM0->PSEL.OUT[0] = P1_11;
    NRF_PWM0->PSEL.OUT[1] = P1_12;         
    NRF_PWM0->PSEL.OUT[2] = 0xFFFFFFFF;   // OUT2 nicht genutzt
    NRF_PWM0->PSEL.OUT[3] = 0xFFFFFFFF;   // OUT3 nicht genutzt


    NRF_PWM0->ENABLE      = 1;
    NRF_PWM0->MODE        = PWM_MODE_UPDOWN_Up;                     // Edge-aligned
    NRF_PWM0->PRESCALER   = PWM_PRESCALER_PRESCALER_DIV_1;          // 16 MHz
    NRF_PWM0->COUNTERTOP  = 16000;                                // 1 kHz
    
    NRF_PWM0->DECODER =
        (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |
        (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);

    NRF_PWM0->SEQ[0].PTR      = (uint32_t)pwm_seq;
    NRF_PWM0->SEQ[0].CNT      = 4;       // Pflicht: 4 Werte für OUT0..OUT3
    NRF_PWM0->SEQ[0].REFRESH  = 0;       // jede Periode Werte neu lesen
    NRF_PWM0->SEQ[0].ENDDELAY = 0;

    
    NRF_PWM0->LOOP   = 0xFFFF;                      // maximale Wiederholzahl
    NRF_PWM0->SHORTS = PWM_SHORTS_LOOPSDONE_SEQSTART0_Msk;

    NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

void BLEhandler()
{
    if (write32Char.written()) 
    {
        const uint8_t* data = write32Char.value();
        uint8_t id = data[0];
        uint32_t val = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);

        //-------------------------------------------------------------------------------------
        WEBLOG("RX id=%u val=%lu\n", (unsigned)id, (unsigned long)val);
        //-------------------------------------------------------------------------------------

        switch (id)
        {
        case 0:
            timerOn = val;
            break;
        case 1:
            timerDim = val;
            break;
        case 2:
            brightness = val;
            break;
        case 3:
            lightColor = val;
            break;
        case 4:
            batteryLevel = val;
            break;
        
        case 99:
            sendNotify(0, timerOn);
            sendNotify(1, timerDim);
            sendNotify(2, brightness);
            sendNotify(3, lightColor);
            sendNotify(4, batteryLevel);
            return;
        }
        // Echo back with same id and value
        sendNotify(id, val);
    } 
}

void setupEncoder()
{
    pinMode(ENC_A_PIN, INPUT_PULLUP); 
    pinMode(ENC_B_PIN, INPUT_PULLUP);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

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

void setupBatMan()
{
  pinMode(VBAT_PIN, INPUT);  //Bat read
  pinMode(P0_14, OUTPUT);
  pinMode(P0_13, OUTPUT);
  pinMode(P1_13, INPUT_PULLUP);
  digitalWrite(P0_14, LOW); //enable Bat read on P0_31
  digitalWrite(P0_13, LOW); //set charging current high
  //digitalWrite(P1_13, HIGH);

  analogReference(AR_INTERNAL2V4);
  analogReadResolution(12);
  analogAcquisitionTime(AT_40_US);
}

uint32_t readBat()
{
    uint16_t raw = analogRead(VBAT_PIN);

    uint32_t ADCMedianNow = (uint16_t)ADCMedian.AddValue((int)raw);

    int32_t BatPcnt = map((long)ADCMedianNow, 1800L, 2360L, 0L, /*9771L*/100L);
    if (BatPcnt < 0) BatPcnt = 0;
    if (BatPcnt > 100) BatPcnt = 100;
    sendNotify(4, BatPcnt);
    
    return(ADCMedianNow);
}

void setup() 
{
    
    timerOn = flash[0];
    timerDim = flash[1];
    brightness = flash[2];
    lightColor = flash[3];
    brightness = 50;

    setupPWM();
    setupEncoder();
    setupBLE();
    setupBatMan();

    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), wakeup, FALLING);
}

void loop() 
{
    static bool firstConnect = true, fallingEdge = true, noShutDown = false;
    static uint32_t fallTime = 0, noShutDownTime = 0, batReadTime = 0;
    uint32_t ms = millis();
    bool ButtonPressed = ! digitalRead(BUTTON_PIN);

    BLEDevice central = BLE.central();

    pwm_seq[0] = brightness * 63;
    pwm_seq[1] = lightColor * 63;
    NRF_PWM0->TASKS_SEQSTART[0] = 1;

    if(central&&firstConnect)
        firstConnect = false;

    if(!firstConnect)
    { 
        if(central.connected()) 
            BLEhandler();
        else
            firstConnect = true;
    }
    
    if(NRF_QDEC->EVENTS_REPORTRDY)
    {
        NRF_QDEC->TASKS_READCLRACC = 1;
        if(ButtonPressed)
        {
            lightColor += NRF_QDEC->ACCREAD + NRF_QDEC->ACCDBLREAD;
            sendNotify(3, lightColor);
            noShutDown = true;
            noShutDownTime = millis();
        }
        else
        {
            brightness += NRF_QDEC->ACCREAD + NRF_QDEC->ACCDBLREAD;
            sendNotify(2, brightness);
        }
        NRF_QDEC->EVENTS_REPORTRDY = 0; // Clear event flag
    }

    if(ButtonPressed&&fallingEdge)
    {
        fallTime = millis();
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
    /*if(ms-batReadTime>100)
        digitalWrite(P1_11, LOW);*/
    if(ms-batReadTime>200)
    {
        static uint8_t i=0;
        //digitalWrite(P1_11, HIGH);
        
        if(i++>50)
        {
            //sendNotify(4, readBat());
            WEBLOG("ADC raw: %u \n", readBat());
            i=0;
        }
        else
            readBat();
        
        batReadTime = ms;
    }
    if(brightness>255)
        shutDown(completely);
}
