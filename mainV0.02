#include <Arduino.h>
#include <ArduinoBLE.h>


#define WARM_PIN P1_12
#define WARMER_PIN P1_11

#define BUTTON_PIN P0_3   // D1
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

uint16_t brightness=0, lightColor=0, batteryLevel;
uint32_t timerOn=0, timerDim=0;
uint32_t* flash = (uint32_t*)FlashAddress;
uint16_t pwm_seq[2] = {0,0};


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

    // Add service
    BLE.addService(myService);

    // Set initial values
    uint8_t initial[5] = {0, 0, 0, 0, 0};
    write32Char.writeValue(initial, 5);
    notifyChar.writeValue(initial, 5);

    // Start advertising
    BLE.advertise();
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

    BLE.end();
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    NRF_NVMC->ERASEPAGE = FlashAddress;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    flash[0] = (lightColor<<16) | brightness;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    flash[1] = timerOn;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    flash[2] = timerDim;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    
    NRF_POWER->SYSTEMOFF = 1;
}

void setupPWM()
{
    NRF_PWM0->PSEL.OUT[0] = (WARM_PIN << PWM_PSEL_OUT_PIN_Pos) |
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    NRF_PWM0->PSEL.OUT[1] = (WARMER_PIN << PWM_PSEL_OUT_PIN_Pos) |
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
    NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    NRF_PWM0->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    NRF_PWM0->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_1 << PWM_PRESCALER_PRESCALER_Pos);
    NRF_PWM0->COUNTERTOP = (16000 << PWM_COUNTERTOP_COUNTERTOP_Pos); //1 msec
    NRF_PWM0->LOOP = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
    NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |
                        (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    NRF_PWM0->SEQ[0].PTR = ((uint32_t)(pwm_seq) << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM0->SEQ[0].CNT = ((sizeof(pwm_seq) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    NRF_PWM0->SEQ[0].REFRESH = 0;
    NRF_PWM0->SEQ[0].ENDDELAY = 0;
    NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

void BLEhandler()
{
    if (write32Char.written()) 
    {
        const uint8_t* data = write32Char.value();
        uint8_t id = data[0];
        uint32_t val = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);

        switch (id)
        {
        case 0:
            brightness = val;
            break;
        case 1:
            lightColor = val;
            break;
        case 2:
            batteryLevel = val;
            break;
        case 3:
            timerOn = val;
            break;
        case 4:
            timerDim = val;
            break;
        case 99:
            sendNotify(0, brightness);
            sendNotify(1, lightColor);
            sendNotify(2, batteryLevel);
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

void setup() 
{
    brightness = flash[0] & 0xFFFF;
    lightColor = (flash[0] >> 16) & 0xFFFF;
    timerOn = flash[1];
    timerDim = flash[2];

    setupPWM();
    setupEncoder();
    setupBLE();

    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), wakeup, FALLING);
}

void loop() 
{
    static bool firstConnect = true, fallingEdge = true, noShutDown = false;
    static uint32_t fallTime = 0;
    uint32_t ms = millis();
    bool ButtonPressed = ! digitalRead(BUTTON_PIN);

    BLEDevice central = BLE.central();

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
            sendNotify(0, lightColor);
            noShutDown = true;
        }
        else
        {
            brightness += NRF_QDEC->ACCREAD + NRF_QDEC->ACCDBLREAD;
            sendNotify(0, brightness);
        }
        NRF_QDEC->EVENTS_REPORTRDY = 0; // Clear event flag
    }

    if(ButtonPressed&&fallingEdge)
    {
        fallTime = millis();
        fallingEdge = false;
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
}
