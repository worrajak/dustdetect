#include <lmic.h>
#include <hal/hal.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <SoftwareSerial.h>
#include <LowPower.h>

SoftwareSerial pmsSerial(3, 4);

float pm25,pm100;
int x;

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

#define RST_PIN -1

SSD1306AsciiWire oled;

#define MY_DEBUG

//byte buf[10];
//byte data_temp[2];
//byte data_humid[1];

static osjob_t sendjob;

const unsigned TX_INTERVAL = 30;      // Time in second to sleep

int channel = 0;
int dr = DR_SF10;  

char buf[50];
#define LENG 23 

long pmat25=0;
long pmat100=0;

bool TX_done = false;
bool next = false;

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C #define BME280_ADDRESS                (0x76)

static const PROGMEM u1_t NWKSKEY[16] = { 0x82, 0x00, 0x89, 0x0C, 0x6F, 0xEC, 0xF7, 0xB4, 0x25, 0x55, 0x44, 0xBC, 0xE2, 0x52, 0x15, 0xAE };
static const u1_t PROGMEM APPSKEY[16] = { 0x7B, 0x9D, 0x3C, 0xD7, 0xA5, 0xF4, 0xA8, 0x88, 0xD9, 0xD6, 0x78, 0xA6, 0xBC, 0x62, 0x85, 0x8B };
static const u4_t DEVADDR = 0x26041236 ; // <-- Change this address for every node!

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

struct {
  uint8_t Header1[2] = {0x01,0x67};
  char temp[2];
  uint8_t Header2[2] = {0x02,0x68};
  char humid[1];
  uint8_t Header3[2] = {0x03,0x02};
  char pms[2];
  uint8_t Header4[2] = {0x04,0x02};
  char vbat[2];  
}mydata;

int vbat_int;
int pms_int;
float temperature = 0;
float humidity = 0;
uint16_t tempC_int = 0;
uint8_t hum_int = 0;

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {9, 8, 0},
};

void read_pms(){
  int count = 0;
  unsigned char c;
  unsigned char high;  
  while (pmsSerial.available()) {
    c = pmsSerial.read();
    if((count==0 && c!=0x42) || (count==1 && c!=0x4d)){
      //Serial.println("check failed");
      break;
    }
    if(count > 15){
      break;
    }
    else if(count == 4 || count == 6 || count == 8 || count == 10 || count == 12 || count == 14) {
      high = c;
    }
    else if(count == 13){
      pmat25 = 256*high + c;
      pm25 = pmat25;
      pms_int = pm25;
    }
    else if(count == 15){
      pmat100 = 256*high + c;
      pm100 = pmat100;
    }
    count++;
  }
  while(pmsSerial.available()) pmsSerial.read();
}

void onEvent (ev_t ev) {
    //Serial.print(os_getTime());
    //Serial.print(": ");
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
            if (LMIC.txrxFlags & TXRX_ACK) {
              Serial.println(F("Received ack"));
            }
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
                Serial.print("RSSI: ");Serial.println(LMIC.rssi);Serial.print("SNR: ");Serial.println(LMIC.snr);
                oled.println();oled.println();
                oled.print("RSSI: ");oled.print(LMIC.rssi);oled.print(" SNR: ");oled.println(LMIC.snr);
            }
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
          for (int i=0; i<int(TX_INTERVAL/8); i++) {
            // Use library from https://github.com/rocketscream/Low-Power
            LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
          }            
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
    next = true;
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        readData();
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, (unsigned char *)&mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
    TX_done = false;
}

void readData()
{
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    read_pms();
    tempC_int = temperature*10;
    hum_int = humidity*2;
    mydata.temp[0] = tempC_int >> 8;
    mydata.temp[1] = tempC_int;
    mydata.humid[0] =  hum_int ;    
    pms_int = pms_int *100;
    mydata.pms[0] = pms_int >> 8;
    mydata.pms[1] = pms_int;    
    vbat_int = readVcc()*0.1;
    mydata.vbat[0] = vbat_int >> 8;
    mydata.vbat[1] = vbat_int;    

    oled.clear();
    oled.println("PM2.5 TTN102");
    oled.println();
    oled.setCursor(0,2);
    oled.print("T:");oled.print(temperature,1);oled.print(" RH:");oled.print(humidity,1);
    oled.setCursor(0,3);
    oled.print("PM25: ");
    oled.set2X();
    oled.print(pm25,0);
    oled.set1X();
    oled.setCursor(0,5);
    oled.print("Vbat: ");
    oled.print(readVcc()*0.001,2);

#ifdef MY_DEBUG  
    Serial.print(temperature);Serial.print(" ");Serial.print(humidity);Serial.print(" ");Serial.print(readVcc());Serial.print(" ");Serial.println(pms_int);
#endif
}

void setup() {
  byte i;
  uint8_t j,a, result;
  float x;
  bool status;    
    
   Serial.begin(9600);
   Serial.println("starting");
   pmsSerial.begin(9600);
    
   status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
    
   Serial.println("BME280 found!!!");     

#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

   oled.setFont(Adafruit5x7);
   
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly 
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 923600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 923800000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band    
    LMIC_setupChannel(4, 924000000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band    
    LMIC_setupChannel(5, 924200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band 
    LMIC_setupChannel(6, 924400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band          
    LMIC_setupChannel(7, 924600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band    

    // Disable link check validation
       LMIC_setLinkCheckMode(0);
    // LMIC_setLinkCheckMode(1);

    LMIC.dn2Dr = DR_SF9;
    // Set data rate and transmit power (note: txpow seems to be ignored by the library)

    //forceTxSingleChannelDr();  
    LMIC_setDrTxpow(dr,14);

    //do_send(&sendjob);
    next = true;
    Serial.println("Leave setup...");
}

void loop() {
 if (next == false) {  
    os_runloop_once();
  }else {    
    next = false;
    do_send(&sendjob);
 }
}

