 #include <Arduino.h>
#include <Esp.h>

#include <../lib/IBM/src/lmic.h>
#include <../lib/IBM/src/hal/hal.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Ticker.h>

#define GET_LORA_APPEUI 0xB0
#define GET_LORA_DEVEUI 0xB1
#define GET_LORA_APPKEY 0xB2

#define SET_LORA_APPEUI 0xA0
#define SET_LORA_APPKEY 0xA2

#define FACTORY_RESET 0xAF

#define PUBLISH_MEASUREMENTS 0xC0

#define REED_ACTIVE 5
#define REED_READ 13


// TX INTERVAL LORA
#define TX_interval 35000
/*
#define APPEUI_DEF {0xDB, 0x16, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70}
//#define DEVEUI_DEF {0x05, 0x70, 0xF0, 0xBF, 0x71, 0x3C, 0x1F, 0x15}
//#define DEVEUI_DEF {0x00, 0x1F, 0x3C, 0x71, 0xCD, 0xE0, 0x10, 0x00}
#define DEVEUI_DEF {0x00, 0x10, 0xE0, 0xCD, 0x71, 0x3C, 0x1F, 0x18}
//18 1f 3c 71 cd e0 10 00
// 00 10 E0 CD 71 3c 1F 18
#define APPKEY_DEF {0x06, 0x26, 0x35, 0x77, 0xC3, 0xBB, 0xC9, 0xEB, 0x2F, 0xEF, 0x99, 0x13, 0x5E, 0xF0, 0xF6, 0xA5}
*/

//   buchutest Config
#define APPEUI_DEF {0x64, 0x47, 0x1d, 0xa3, 0xe1, 0x71, 0x99, 0x13}
#define DEVEUI_DEF {0x64, 0x47, 0x1d, 0xa3, 0xe1, 0x71, 0x99, 0x13}//LSB 
//#define APPKEY_DEF {0x53, 0x6c, 0xaa, 0x19, 0x27, 0x53, 0xdc, 0x21, 0x25, 0xc2, 0xa7, 0xc3, 0x50, 0xf7, 0x37, 0xd6} //LSB
#define APPKEY_DEF {0xd6, 0x37, 0xf7, 0x50, 0xc3, 0xa7, 0xc2, 0x25, 0x21, 0xdc, 0x53, 0x27, 0x19, 0xaa, 0x6c, 0x53} //MSB



#define MEASURE_TIME 10
#define BATT_READ_TIME 30

#define BT_TIMEOUT 30000
#define BT_NAME "GESCO2"
#define BT_PIN "3217"
#define BT2_TX 17
#define BT2_RX 16

#define BT_POWER 15

#define CCS_NWAKE 23

static u1_t APPEUI[8] = APPEUI_DEF;
void os_getArtEui(u1_t *buf) { memcpy(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = DEVEUI_DEF;
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static u1_t APPKEY[16] = APPKEY_DEF;
void os_getDevKey(u1_t *buf) { memcpy(buf, APPKEY, 16); }

static osjob_t sendjob;

bool lora_joined = false;
bool sent = false;
bool start_send = false;
bool configuring = false;
bool initialized = false;
unsigned long configuring_time;

uint8_t mydata[50] = "FirstSend";
String data = "";
String generalData = "";
void do_send(osjob_t *j);

bool serialArrived = false;
unsigned int countMessage = 0;
unsigned long lastmillis = 0;
unsigned int msgInQueue = 1;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, LMIC_UNUSED_PIN},
    //.rxtx_rx_active = 0,
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        if (msgInQueue>0){
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
        msgInQueue--;
        }
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
void reset_data(){

    for(int i = 0 ; i< sizeof(mydata); i++){
        mydata[i] = 0;
    }
    data = "";
    generalData = "";
    countMessage = 0;
}

void rx(osjobcb_t func) {
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  os_radio(RADIO_RXON);
  Serial.println("RX");
}

static void rx_func (osjob_t* job) {
  Serial.println(F("Received "));
  Serial.print(LMIC.dataLen);
  Serial.println(F(" bytes of payload"));
  Serial.print("[");
  for (int i=0; i < LMIC.dataLen; i++)
  {
    Serial.print(LMIC.frame[LMIC.dataBeg+i]);
    Serial.print(" ");
  }
  Serial.println("]");

  // Restart RX
  rx(rx_func);
}

void setup() {
    Serial.begin(9600);
    Serial.println(F("Starting"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
     //setup initial job
    os_setCallback(&txjob, rx_func);
}

void loop() {
    os_runloop_once();

    do_send(&sendjob);
        
}