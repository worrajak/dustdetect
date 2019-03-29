/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <DHT.h>

#define DHTTYPE DHT22
#define DHTPIN  2
DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for ESP8266

int measurePin = A0;
int ledPower = 3;

unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C #define BME280_ADDRESS                (0x76)
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

int Log_flag=0;
long lastMsg = 0;
float temp = 0.0;
float hum = 0.0;
float diff = 1.0;
float temperature = 0;
float humidity = 0;

bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}

void setup() {
    pinMode(ledPower,OUTPUT);
    Serial.begin(9600);
    //Serial.println(F("Dust test"));
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  // text display tests
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    bool status;
 //   display.drawBitmap(13, 26,  battery, 24, 16, 1); 
  display.println("Dust test");
  display.display();
  delay(1000);
/*    
    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
    
    Serial.println("-- Default Test --");
    delayTime = 5000;
*/
    Serial.println();
}

void loop() { 

    float newTemp = dht.readTemperature();
    float newHum = dht.readHumidity();
  
    if (checkBound(newTemp, temp, diff)) {
      temperature = newTemp;
      Serial.print("New temperature:");
      Serial.println(String(temperature).c_str());
    }

    if (checkBound(newHum, hum, diff)) {
      humidity = newHum;
      Serial.print("New humidity:");
      Serial.println(String(humidity).c_str());
    }

  digitalWrite(ledPower,LOW);
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin);

  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH);
  delayMicroseconds(sleepTime);

  calcVoltage = voMeasured*(5.0/1024);
  dustDensity = 0.17*calcVoltage-0.1;

  if ( dustDensity < 0)
  {
    dustDensity = 0.00;
  }
    
    printValues();
    delay(2000);
}

void printValues() {
    //Serial.print("Temperature = ");
    //Serial.print(temperature);
    //Serial.print(bme.readTemperature());
    //Serial.println(" *C");
    //Serial.print("Pressure = ");
    //Serial.print(bme.readPressure() / 100.0F);
    //Serial.println(" hPa");
    //Serial.print("Approx. Altitude = ");
    //Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    //Serial.println(" m");
    //Serial.print("Humidity = ");
    //Serial.print(humidity);
    //Serial.print(bme.readHumidity());
    //Serial.println(" %");
    //Serial.println();

    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Dust Sensor");
    display.setCursor(0,20);
    display.print("Temperature: ");
    //display.println(bme.readTemperature());
    display.println(temperature);
    display.print("Humidity: ");
    display.println(humidity);  
    //display.println(bme.readHumidity());
    //display.print("Pres.= ");
    //display.print(bme.readPressure() / 100.0F);
    //display.println(" hPa");
    //display.print("Alt= ");
    //display.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    //display.println(" m");  
    //display.print("Raw Signal: ");
    //display.println(voMeasured); 
    //display.print("Voltage:"); 
    //display.println(calcVoltage);  
    display.print("Dust Density:");
    display.println(dustDensity);
    display.display();
}
