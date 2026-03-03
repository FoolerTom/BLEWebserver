//#include <Arduino.h>
#include <bluefruit.h> // Adafruit nRF52 BLE library

#include <MedianFilterLib.h>

//----------------------------------------------------------------------------------------
#include <stdarg.h>   // <- Wichtig für va_list, va_start, va_end
#include <stdio.h>    // optional: für vsnprintf()
//----------------------------------------------------------------------------------------



// pin assignments – use Arduino "Dx" names where possible so the
// Adafruit/XIAO board mapping is correct.  P0_x/P1_x constants still exist
// but may not match the physical pinout of the new board, so use the
// Arduino defines for the pins actually used by the hardware.
#define WARM_PIN        11      // unchanged – warm LED output
#define WARMER_PIN      12      // unchanged – warmer LED output
#define VBAT_PIN        31      // analogue battery sense
#define BUTTON_PIN      3         // formerly P0_3
// Quadrature Encoder Pins (Hardware QDEC)
#define ENC_A_PIN       29         // formerly P0_29
#define ENC_B_PIN       28         // formerly P0_28
#define BAT_EN_PIN      14     // enable voltage divider for battery measurement
#define BAT_DISCON_PIN  13     // disconnect battery (pulls low to disconnect)
#define CHARGE_PIN      13     // control charging current
#define CHARGE_INDI_PIN 17

#define IN_UP           12
#define IN_DOWN         4
#define INP             0
#define OUTP            3
#define EN_INTERRUPT    (3<<16)

#define FlashAddress   0x000F0000

#define SampleNumber    20


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

// forward declaration for write callback used by setupBLE
void write32_cb(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len);

uint32_t brightness=0, lightColor=0, batteryLevel=0;
uint32_t timerOn=0, timerDim=0;
uint32_t* flash = (uint32_t*)FlashAddress;
uint16_t pwm_seq[4] = {4000,12000,0,0};
int16_t adc_buffer[50], saadc_buffer;
uint32_t ble_connect_time = 0;  // timestamp when BLE started
bool ble_enabled = true, buffer_ready = false;        // track if BLE is active

MedianFilter<uint32_t> ADCMedian(SampleNumber);

const bool completely = true;


//---------------------------------------------------------------------------------------------------------------
#if WEBLOG_ENABLE
static void weblog_write(const uint8_t* data, size_t len) {
  // BLE Notify in Chunks (typisch 20..200B; wir nehmen 180 als konservativ)
  const size_t CHUNK = 180;
  size_t off = 0;
  while (off < len) {
    size_t n = (len - off) < CHUNK ? (len - off) : CHUNK;
    logChar.write(data + off, n);
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
  #define WEBLOG(...)       do{}while(0)
  #define WEBLOGLN(s)       do{}while(0)
#endif
//---------------------------------------------------------------------------------------------------------------

//void wakeup(){}

void sendNotify(uint8_t id, uint32_t val) 
{
    uint8_t response[5] = {id, (uint8_t)val, (uint8_t)(val >> 8), (uint8_t)(val >> 16), (uint8_t)(val >> 24)};
    notifyChar.write(response, 5);
    // Bluefruit notify requires data/length arguments
    notifyChar.notify(response, 5);
}

// write callback for Bluefruit – replaces the earlier BLEhandler()
void write32_cb(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
    
    if (len < 1) return;
    uint8_t id = data[0];
    uint32_t val = 0;
    if (len >= 5) {
        val = (uint32_t)data[1] |
              ((uint32_t)data[2] << 8) |
              ((uint32_t)data[3] << 16) |
              ((uint32_t)data[4] << 24);
    }
    Serial.print(val);
    Serial.print("\t");
    Serial.println(id);
    //-------------------------------------------------------------------------------------
    //WEBLOG("RX id=%u val=%lu\n", (unsigned)id, (unsigned long)val);
    //-------------------------------------------------------------------------------------

    switch (id) {
        case 0: timerOn = val;      break;
        case 1: timerDim = val;     break;
        case 2: brightness = val;   break;
        case 3: lightColor = val;   break;
        case 4: batteryLevel = val; break;
        case 99:
            sendNotify(0, timerOn);
            sendNotify(1, timerDim);
            sendNotify(2, brightness);
            sendNotify(3, lightColor);
            sendNotify(4, batteryLevel);
            return;
    }
    // echo back
    sendNotify(id, val);
}

void setupBLE()
{
    // start SoftDevice and BLE stack
    Bluefruit.begin();
    ble_connect_time = millis();  // record when BLE started
    Bluefruit.setName("Nachtlicht");
    Bluefruit.setTxPower(4);
    Bluefruit.autoConnLed(false);

    // configure service
    myService.begin();

    write32Char.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
    write32Char.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    write32Char.setFixedLen(5);
    write32Char.setWriteCallback(write32_cb);
    write32Char.begin();

    notifyChar.setProperties(CHR_PROPS_NOTIFY);
    notifyChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    notifyChar.setFixedLen(5);
    notifyChar.begin();

    #if WEBLOG_ENABLE
    logChar.setProperties(CHR_PROPS_NOTIFY);
    logChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    logChar.setFixedLen(200);
    logChar.begin();
    uint8_t z = 0;
    logChar.write(&z, 1);
    #endif

    // advertising
    Bluefruit.Advertising.addService(myService);
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addName();               // include the name in the advertisement
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.start(0); // no timeout

    //---------------------------------------------------------
    WEBLOGLN("BLE bereit, Werbung gestartet.");
    //---------------------------------------------------------

    // automatically push current values when a new central connects
    Bluefruit.Periph.setConnectCallback([](uint16_t conn_hdl){
        WEBLOG("central connected, sending initial state\n");
        sendNotify(0, timerOn);
        sendNotify(1, timerDim);
        sendNotify(2, brightness);
        sendNotify(3, lightColor);
        sendNotify(4, batteryLevel);
    });
}

void shutDown(bool batteryDisconnect = false)
{
    //__disable_irq();
    Serial.println("Shut down");
    Serial.flush();  // ensure serial output is written before sleep
    delay(50);  // give time for last output to transmit

    // disable SAADC interrupt early
    //NVIC_DisableIRQ(SAADC_IRQn);
    /*NRF_SAADC->TASKS_STOP = 1;
    NRF_SAADC->ENABLE = 0;

    NRF_QDEC->TASKS_STOP = 1; 
    NRF_QDEC->ENABLE = 0; 
    NRF_PWM0->ENABLE = 0;
    NRF_PWM1->ENABLE = 0; 
    NRF_UART0->ENABLE = 0; 
    NRF_TWIM0->ENABLE = 0; 
    NRF_SPIM0->ENABLE = 0;

    NRF_TIMER0->TASKS_STOP = 1;
    NRF_TIMER1->TASKS_STOP = 1;

    NRF_P1->PIN_CNF[WARMER_PIN] = IN_UP;
    NRF_P1->PIN_CNF[WARM_PIN] = IN_UP;
    
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, LOW);
    //__enable_irq();
    Serial.print(NRF_NVMC->CONFIG);
    Serial.println(NRF_NVMC->READY);
    
    //delay(500);
    //__disable_irq();
    // make sure BLE stack is shut down
    // disconnect all connections gracefully*/
    /*Serial.println("A");
    Bluefruit.disconnect(BLE_CONN_HANDLE_ALL);
    Serial.println("B");
    Bluefruit.Advertising.stop();*/
    Serial.println("C");
    Serial.flush();
    delay(100);
    /*__disable_irq();
    sd_flash_page_erase(FlashAddress);
    sd_flash_write((uint32_t*)FlashAddress, (uint32_t*)timerOn, 1);*/
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
    //__enable_irq();
    Serial.println("D");
    Serial.flush();
    delay(100);
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        pinMode(LED_GREEN, OUTPUT);
        digitalWrite(LED_GREEN, LOW);
        Serial.print(NRF_NVMC->CONFIG);
        Serial.println(NRF_NVMC->READY);
        Serial.flush();
        delay(500);
    }
    //__disable_irq();
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
    //delay(100);
    digitalWrite(LED_BLUE, HIGH);
    NRF_P0->PIN_CNF[BUTTON_PIN] |= EN_INTERRUPT;
    
    
    //Serial.println("now");
    //delay(5000);
    __enable_irq();
    if(batteryDisconnect)   //the battery gets disconnected. Wakeup only through USB charching
        NRF_P1->PIN_CNF[BAT_DISCON_PIN] = IN_DOWN;
        //pinMode(BAT_DISCON_PIN, INPUT_PULLDOWN);*/
    Serial.flush();
    delay(100);
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, LOW);
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

void setupEncoder()
{
    //pinMode(ENC_A_PIN, INPUT_PULLUP); 
    //pinMode(ENC_B_PIN, INPUT_PULLUP);
    //pinMode(BUTTON_PIN, INPUT_PULLUP);
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

void setupBatMan()
{
    NRF_P0->PIN_CNF[VBAT_PIN] = INP;
    NRF_P0->PIN_CNF[CHARGE_INDI_PIN] = INP;
    NRF_P0->PIN_CNF[BAT_EN_PIN] = OUTP;
    NRF_P0->PIN_CNF[CHARGE_PIN] = OUTP;
    NRF_P1->PIN_CNF[BAT_DISCON_PIN] = IN_UP;
  /*pinMode(VBAT_PIN, INPUT);  //Bat read
  pinMode(CHARGE_INDI_PIN, INPUT);
  pinMode(BAT_EN_PIN, OUTPUT);
  pinMode(CHARGE_PIN, OUTPUT);
  pinMode(BAT_DISCON_PIN, INPUT_PULLUP);*/
    NRF_P0->OUTCLR = (1<<BAT_EN_PIN)|(1<<CHARGE_PIN);
  /*digitalWrite(BAT_EN_PIN, LOW); //enable Bat read on VBAT_PIN
  digitalWrite(CHARGE_PIN, LOW); //set charging current high*/
    NRF_SAADC->ENABLE = 0;
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;
    NRF_SAADC->CH[0].CONFIG = 0x00050200;
    NRF_SAADC->CH[0].PSELP = 8; //AIN7 = P0.31 = VBAT_PIN
    NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;
    NRF_SAADC->RESULT.PTR = (uint32_t)&saadc_buffer;
    NRF_SAADC->RESULT.MAXCNT = 1;


    //NRF_SAADC->SAMPLERATE = (1<<12)|2000;
    
    /*
     * enable interrupt on buffer full (EVENTS_END) so we can
     * process the data without polling in loop().
     */
    //
    //NVIC_EnableIRQ(SAADC_IRQn);
    Serial.println("A");
    
    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->EVENTS_RESULTDONE = 0;

    NRF_SAADC->INTENSET = (1<<3);
    NVIC_EnableIRQ(SAADC_IRQn);
    // finally enable peripheral
    NRF_SAADC->ENABLE = 1;

    // start the SAADC once; later loops will restart before each sample
    NRF_SAADC->TASKS_START = 1;

    // take first sample immediately
    NRF_SAADC->TASKS_SAMPLE = 1;

    Serial.println("ADC initialised");

  /*analogReference(AR_INTERNAL_2_4);
  analogReadResolution(12);*/
  // analogAcquisitionTime not available on this core
}

/*uint32_t*/void readBat()
{
    /*static uint8_t sendCnt = 0;
    uint16_t raw = analogRead(VBAT_PIN);

    uint32_t ADCMedianNow = (uint16_t)ADCMedian.AddValue((int)raw);

    int32_t BatPcnt = map((long)ADCMedianNow, 1800L, 2360L, 0L, /*9771L*//*100L);
    if (BatPcnt < 0) BatPcnt = 0;
    if (BatPcnt > 100) BatPcnt = 100;
    batteryLevel = BatPcnt;          // remember latest level
    if(sendCnt++>10)
    {
        sendNotify(4, BatPcnt|(!digitalRead(CHARGE_INDI_PIN)<<31));
        sendCnt = 0;
    }
    return(ADCMedianNow|(!digitalRead(CHARGE_INDI_PIN)<<31));*/
    uint32_t BatBuffer = 0;
    /*NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->TASKS_STOP = 0;*/
    /*for(uint8_t i=0; i<SampleNumber; i++)
        BatBuffer += adc_buffer[i];
    BatBuffer /= SampleNumber;*/
    BatBuffer = ADCMedian.GetFiltered();
    uint32_t BatPcnt = map(BatBuffer, 1800L, 2360L, 0L, 100L);
    if (BatPcnt < 0) BatPcnt = 0;
    if (BatPcnt > 100) BatPcnt = 100;
    if(!(NRF_P0->IN & (1<<CHARGE_INDI_PIN)))
        BatPcnt |= 1<<31;
    batteryLevel = BatPcnt;
    sendNotify(4, batteryLevel);    
    //return batteryLevel;
    Serial.print("Bat: ");
    Serial.println(BatBuffer);
    WEBLOG("ADC raw: %u \t %u\n", batteryLevel, BatBuffer);
    //NRF_SAADC->TASKS_SAMPLE = 1;
}

extern "C" void SAADC_IRQHandler(void)
{
    // IMPORTANT: No Serial or blocking operations in ISR!
    // ISR must be fast and non-blocking
    static uint8_t buf_cnt = 0;
    if (NRF_SAADC->EVENTS_RESULTDONE)
    {
        NRF_SAADC->EVENTS_RESULTDONE = 0;
        (uint16_t)ADCMedian.AddValue(saadc_buffer);
        NRF_SAADC->TASKS_START = 1;
        /*adc_buffer[buf_cnt++] = saadc_buffer;
        if(buf_cnt>=SampleNumber)
        {
            buf_cnt = 0;
            buffer_ready = true;
        }*/
       /*if(buf_cnt++>=10/*SampleNumber)
       {
            buf_cnt = 0;
            buffer_ready = true;
       }*/
    }
}

void setupTimerPpi()
{
    NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER1->PRESCALER = 4; // 1 MHz (16 MHz / 2^4)

    NRF_TIMER1->CC[0] = 100000; // 100 ms

    NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;

    // PPI: TIMER1 COMPARE[0] -> SAADC SAMPLE
    NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TIMER1->EVENTS_COMPARE[0];
    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_SAADC->TASKS_SAMPLE;
    NRF_PPI->CHENSET = PPI_CHENSET_CH0_Msk;

    NRF_TIMER1->TASKS_START = 1;
}

void setup() 
{
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_BLUE, LOW);
    timerOn = flash[0];
    timerDim = flash[1];
    brightness = flash[2];
    lightColor = flash[3];
    brightness = 50;

    Serial.begin(115200);
    //while(!Serial);
    Serial.println("Start");

    setupPWM();
    setupEncoder();
    setupBLE();
    setupBatMan();
    setupTimerPpi();

    //attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), wakeup, FALLING);
}

void loop() 
{
    static bool fallingEdge = true, noShutDown = false;
    static uint32_t fallTime = 0, noShutDownTime = 0, batReadTime = 0;
    uint32_t ms = millis();
    bool ButtonPressed = !(NRF_P0->IN&(1<<BUTTON_PIN));

    pwm_seq[0] = brightness * 63;
    pwm_seq[1] = lightColor * 63;
    NRF_PWM0->TASKS_SEQSTART[0] = 1;

    if(NRF_QDEC->EVENTS_REPORTRDY)
    {
        NRF_QDEC->TASKS_READCLRACC = 1;
        Serial.println(ButtonPressed);
        if(ButtonPressed)
        {
            lightColor += NRF_QDEC->ACCREAD + NRF_QDEC->ACCDBLREAD;
            sendNotify(3, lightColor);
            noShutDown = true;
            noShutDownTime = millis();
            Serial.println(lightColor);
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
        digitalWrite(WARMER_PIN, LOW);*/
    if(ms-batReadTime>5000)
    {
        readBat();
        /*if(buffer_ready)
        {
            buffer_ready = false;
            readBat();
            
        }
        NRF_SAADC->TASKS_START = 1;
        NRF_SAADC->TASKS_SAMPLE = 1;*/
        /*if(NRF_SAADC->EVENTS_STOPPED)
            NRF_SAADC->TASKS_START = 1;
        NRF_SAADC->EVENTS_RESULTDONE = 0;*/
        //NRF_SAADC->TASKS_SAMPLE = 1;
        /*static uint8_t i=0;
        //digitalWrite(WARMER_PIN, HIGH);
        
        if(i++>5)
        {
            //sendNotify(4, readBat());
            
            i=0;
            Serial.print(digitalRead(BUTTON_PIN));
            Serial.print(digitalRead(ENC_A_PIN));
            Serial.println(digitalRead(ENC_B_PIN));
        }
        else
            readBat();*/
        /*Serial.print(NRF_SAADC->EVENTS_STARTED);
        Serial.print(NRF_SAADC->EVENTS_STOPPED);
        Serial.println(NRF_SAADC->EVENTS_RESULTDONE);*/
        Serial.println(NRF_NVMC->READY);
        /*Serial.println(adc_buffer[0]);
        Serial.println(adc_buffer[1]);
        Serial.println(adc_buffer[49]);*/
        batReadTime = ms;
    }
    /*if(NRF_SAADC->EVENTS_END)
    {
        //NRF_SAADC->EVENTS_DONE = 0;
        readBat();
        WEBLOG("ADC raw: %u \n", batteryLevel);
    }*/
    
    // BLE timeout: if no connection for 60 seconds, disable BLE to save power
    if (ble_enabled && !Bluefruit.connected()) 
    {
        if ((ms - ble_connect_time) > 60000) 
        {  // 60 seconds
            Bluefruit.Advertising.stop();
            ble_enabled = false;
        }
    }
    
    if(brightness>255)
        shutDown(completely);
}
