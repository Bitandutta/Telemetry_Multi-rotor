#include <Wire.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_ADS1X15.h>

#define baudRate 9600
#define BME_Add 0x76
#define airPressure 1008
#define maxAltChange 50.0  // 50 metre
#define latitudeFactor 100000000
#define longitudeFactor 10000000
#define dataDivisionFactor 100
#define voltageOffset 0.25

Adafruit_BMP280 bmp;  // I2C
Adafruit_ADS1115 ads;

RF24 radio(9, 10);  // CE, CSN
const byte address[6] = "00001";

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(4, 3);  // GPS_RXPin = 3, GPS_TXPin = 4

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
typedef struct Data_Package {

  unsigned long latitude;   // 4 byte
  unsigned long longitude;  // 4 byte
  byte satelliteCount;      // 1 byte
  byte polarity;            // 1 byte      // 1-> +lat,+long;  2-> +lat,-long;  3-> -lat,+long;  4-> -lat,-long
  byte distQuotient;        // 1 byte
  byte distRemainder;       // 1 byte
  byte altQuotient;         // 1 byte
  byte altRemainder;        // 1 byte
  byte velocity;            // 1 byte
  byte Hour;                // 1 byte
  byte Minute;              // 1 byte
  byte tempQuotient;        // 1 byte
  byte tempRemainder;       // 1 byte
  byte altPolarity;         // 1 byte
  byte AltitudeQuotient;    // 1 byte
  byte AltitudeRemainder;   // 1 byte
  byte homeLock;            // 1 byte
  byte bvQuotient;          // 1 byte
  byte bvRemainder;         // 1 byte
  byte hdop;                // 1 byte
                            // Total 26 bytes
} Data;
Data d;  // Create a variable with the above structure

static double HomeLat = 22.3461406, HomeLong = 88.4389681, currentAlt, lastAlt, homeAltitude;
int i, batVol;
unsigned int t;

void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(A0, INPUT);
  Serial.begin(baudRate);
  ss.begin(baudRate);
  bmp.begin(BME_Add);

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();

  ads.setGain(GAIN_TWOTHIRDS);
  ads.begin();

  for (i = 1; i < 5; i++) {
    homeAltitude = bmp.readAltitude(airPressure);
    delay(50);
  }
  Serial.print("Home Altitude: ");
  Serial.println(homeAltitude);
  lastAlt = homeAltitude;
  //lastBatVol = ((int)(ads.computeVolts(ads.readADC_SingleEnded(0)) * 100.0));
  d.homeLock = 0;
}

void loop() {

  updateData();
  radio.write(&d, sizeof(Data));
  updateVoltage();

  if (digitalRead(2)==LOW) {

    Serial.println("Button Pressed");
    if (gps.location.isValid()) {
      Serial.println("Home updated!");
      HomeLat = gps.location.lat();
      HomeLong = gps.location.lng();
      if(d.homeLock==255) d.homeLock=1;
      else d.homeLock++;
    }
    
  }

  //displaySerial();

  smartDelay(1000);
}

void updateVoltage() {
  batVol = ((int)(ads.computeVolts(ads.readADC_SingleEnded(0)) * 100.0));
  //if(abs(lastBatVol - batVol) < voltageOffset) lastBatVol=batVol;
  //else batVol=lastBatVol;
  d.bvQuotient = batVol / dataDivisionFactor;
  d.bvRemainder = batVol % dataDivisionFactor;
}

void displaySerial() {
  Serial.print("Satellite Count: ");
  Serial.println(d.satelliteCount);

  Serial.print("HDOP: ");
  Serial.print(d.hdop);
  Serial.println();

  Serial.print("Latitude: ");
  Serial.print(d.latitude);
  Serial.print("\tLongitude: ");
  Serial.print(d.longitude);
  Serial.print("\tPolarity: ");
  Serial.println(d.polarity);

  Serial.print("Distance: ");
  Serial.print((d.distQuotient * 100) + d.distRemainder);
  Serial.println(" metre");

  Serial.print("GPS Altitude: ");
  Serial.println((float)(((d.altQuotient * 100) + d.altRemainder) / 100.0));

  Serial.print("Time: ");
  Serial.print(d.Hour);
  Serial.print(":");
  Serial.println(d.Minute);

  Serial.print("Speed: ");
  Serial.print(d.velocity);
  Serial.println("km/hr");

  Serial.print("Temperature: ");
  Serial.println(((float)((d.tempQuotient * 100) + d.tempRemainder)) / 100.0);

  Serial.print("BME Altitude: ");
  Serial.println(((d.altPolarity) ? 1.0 : -1.0) * ((float)((d.AltitudeQuotient * 100) + d.AltitudeRemainder)) / 100.0);

  Serial.print("Home lock: ");
  Serial.println((int)(d.homeLock));

  Serial.print("Voltage: ");
  //Serial.println(((float)(batVol)) / 100.0);
  Serial.println(ads.readADC_SingleEnded(0));

  Serial.println("\n\n");
}



void updateData() {
  t = ((int)((bmp.readTemperature()) * 100.0));
  d.tempQuotient = t / dataDivisionFactor;
  d.tempRemainder = t % dataDivisionFactor;

  currentAlt = bmp.readAltitude(airPressure);

  if (fabs(lastAlt - currentAlt) < maxAltChange) {
    lastAlt = currentAlt;
    i = (int)(currentAlt * 100.0);
    i -= (int)(homeAltitude * 100.0);

    if (i < 0) d.altPolarity = 1;  // negative
    else d.altPolarity = 0;        // positive

    i = abs(i);

    d.AltitudeQuotient = i / dataDivisionFactor;
    d.AltitudeRemainder = i % dataDivisionFactor;
  }

  d.satelliteCount = (byte)(gps.satellites.value());

  if (d.satelliteCount > 0) {
    int x=gps.hdop.value();
    if(x>400) d.hdop=0;
    else if(x<=400 && x>=100) d.hdop = map(x, 100, 400, 100, 0);
    else d.hdop = 100;

    d.Hour = (byte)(gps.time.hour());
    d.Minute = (byte)(gps.time.minute());

    if (gps.location.isValid()) {

      if (gps.location.lat() >= 0 && gps.location.lng() >= 0) d.polarity = 1;
      else if (gps.location.lat() >= 0 && gps.location.lng() < 0) d.polarity = 2;
      else if (gps.location.lat() < 0 && gps.location.lng() >= 0) d.polarity = 3;
      else d.polarity = 4;

      d.latitude = (unsigned long)(gps.location.lat() * latitudeFactor);
      d.longitude = (unsigned long)(gps.location.lng() * longitudeFactor);
      t = (unsigned int)(TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), HomeLat, HomeLong));
      d.distQuotient = t / dataDivisionFactor;
      d.distRemainder = t % dataDivisionFactor;
      if (gps.altitude.isValid()) {
        t = (unsigned int)(gps.altitude.meters() * 100.0);
        d.altQuotient = t / dataDivisionFactor;
        d.altRemainder = t % dataDivisionFactor;
      } else {
        d.altQuotient = 0;
        d.altRemainder = 0;
      }
      if (gps.speed.isValid()) {
        d.velocity = (byte)(gps.speed.kmph());
      } else {
        d.velocity = 0;
      }

    } else {
      d.polarity = 0;
      d.latitude = 0;
      d.longitude = 0;
      d.distQuotient = 0;
      d.distRemainder = 0;
      d.altQuotient = 0;
      d.altRemainder = 0;
      d.velocity = 0;
      d.hdop = 0;
    }
  } else {
    d.polarity = 0;
    d.latitude = 0;
    d.longitude = 0;
    d.distQuotient = 0;
    d.distRemainder = 0;
    d.altQuotient = 0;
    d.altRemainder = 0;
    d.velocity = 0;
    d.Hour = 0;
    d.Minute = 0;
    d.hdop = 0;
  }
}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
