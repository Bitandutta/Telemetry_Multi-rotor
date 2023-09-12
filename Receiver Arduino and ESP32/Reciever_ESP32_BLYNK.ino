// RECIEVER ESP32

#define BLYNK_TEMPLATE_ID "TMPL3TH5cCAxa"
#define BLYNK_DEVICE_NAME "Telemetry"
#define BLYNK_AUTH_TOKEN "BJgoA23Ucmw_WWyIimoqEHjBsdN0vYFu"
#define router "SD_2.4GHz"
#define iphone "Samrat"

char auth[] = BLYNK_AUTH_TOKEN;
char passR[] = "3dprintingandrobotics";
char passP[] = "12345678";

//#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <Wire.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LiquidCrystal_I2C.h>
#define buzzer 13
#define baudRate 9600
#define latitudeFactor 100000000.0
#define longitudeFactor 10000000.0
#define dataDivisionFactor 100
#define INTERVAL 1000L
#define connectionLost 2000
#define batConst 3.09090909
#define batWarning 10.8

BlynkTimer timer;


LiquidCrystal_I2C lcd1(0x27, 20, 4);
LiquidCrystal_I2C lcd2(0x26, 16, 2);

RF24 radio(4, 5);  // 4->CE, 5->CSN
const byte address[6] = "00001";

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

byte degree[] = {
  B00100,
  B01010,
  B00100,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};

byte locked[] = {
  B00100,
  B01010,
  B01010,
  B11111,
  B11011,
  B11011,
  B11111,
  B00000
};
byte notLocked[] = {
  B00100,
  B01010,
  B01000,
  B11111,
  B11011,
  B11011,
  B11111,
  B00000
};
byte hotspot[] = {
  B10100,
  B11100,
  B10100,
  B00011,
  B00100,
  B00010,
  B00001,
  B00110
};
byte WIFI[] = {
  B10001,
  B10101,
  B01110,
  B00000,
  B11101,
  B10000,
  B11001,
  B10001
};
byte standAlone[] = {
  B01100,
  B10000,
  B01000,
  B00100,
  B11010,
  B00101,
  B00111,
  B00101
};


int t, lastLock=0;
float i, j;
String GPS_latitude, GPS_longitude, GPS_satellite, GPS_distance, GPS_altitude, GPS_time, GPS_speed, BME_temp, BME_altitude, BV, Hdop;
unsigned long check, bt, bt2;
bool lost = false;
bool Mode = false, sa = false, first, buzState;


void setup() {
  pinMode(15, INPUT_PULLUP);  // D-Link
  pinMode(2, INPUT_PULLUP);   // iPhone
  pinMode(buzzer, OUTPUT);

  Serial.begin(baudRate);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  lcd1.init();
  lcd1.backlight();
  lcd1.createChar(0, degree);
  lcd1.createChar(1, notLocked);
  lcd1.createChar(2, locked);
  lcd1.createChar(3, hotspot);
  lcd1.createChar(4, WIFI);
  lcd1.createChar(5, standAlone);
  lcd2.init();
  lcd2.backlight();

  lcd1.clear();
  lcd2.clear();
  lcd2.setCursor(0, 0);
  lcd2.print("Telemetry System");
  lcd2.setCursor(0, 1);
  lcd2.print("V-1.0  Dev-3DPaR");
  lcd1.setCursor(0, 0);
  lcd1.print("Connecting Telemetry");
  lcd1.setCursor(4, 1);
  lcd1.print("Please Wait!");
  Mode = false;
  sa = true;
  if (digitalRead(2) == HIGH && digitalRead(15) == LOW) {
    Serial.println("Connect iPhone");
    lcd1.setCursor(3, 3);
    lcd1.print("Connect iPhone");
    Blynk.begin(auth, iphone, passP);
    Mode = true;
    sa = false;
  } else if (digitalRead(2) == LOW && digitalRead(15) == HIGH) {
    Serial.println("Connect your Router");
    lcd1.setCursor(3, 3);
    lcd1.print("Connect Router");
    Blynk.begin(auth, router, passR);
    Mode = true;
    sa = false;
  }

  Blynk.syncAll();

  lcd1.clear();
  if (!sa) {
    lcd1.setCursor(1, 3);
    lcd1.print("Internet Connected");
  } else {
    Serial.println("Standalone Mode");
    lcd1.setCursor(2, 3);
    lcd1.print("Standalone Mode");
  }
  lcd1.setCursor(4, 0);
  lcd1.print("Waiting for");
  lcd1.setCursor(5, 2);
  lcd1.print("Telemetry");
  //delay(1000);

  timer.setInterval(INTERVAL, send_Blynk);
  check = millis();
  first=true;
  bt=0;
  bt2=0;
  buzState=false;
}

void loop() {
  // Check whether there is data to be received
  if (Mode) {
    Blynk.run();
    timer.run();
  }

  if (radio.available()) {
    while (radio.available()) radio.read(&d, sizeof(Data));  // Read the whole data and store it into the 'data' structure
    first=false;
    storeData();
    displayData();
    updateLCD();
    if(lastLock != d.homeLock){
      digitalWrite(buzzer, HIGH);
      bt=millis();
      lastLock=d.homeLock;
    }
    else if(millis()-bt >= 1000) digitalWrite(buzzer, LOW);
    lost = false;
    check = millis();

    float x=((((float)((d.bvQuotient * dataDivisionFactor) + d.bvRemainder)) / 100.0) * batConst);
    if(x<=batWarning){
      if(buzState){
        digitalWrite(buzzer, LOW);
        buzState=false;;
      }
      else {
        digitalWrite(buzzer, HIGH);
        buzState=true;
      }
    }

  } else if (millis() - check > connectionLost && !first) {

    if(digitalRead(buzzer)==HIGH && millis()-bt2>250){
      digitalWrite(buzzer, LOW);
      bt2=millis();
    }
    else if(digitalRead(buzzer)==LOW && millis()-bt2>1500){
      digitalWrite(buzzer, HIGH);
      bt2=millis();
    }

    if (lost == false) {
      Serial.println("\n\nConnection Lost!");
      lcd1.clear();
      lcd1.setCursor(2, 1);
      lcd1.print("Connection Lost!");
      lcd2.clear();
      lcd2.setCursor(2, 0);
      lcd2.print("No Telemetry");
      lcd2.setCursor(4, 1);
      lcd2.print("Waiting!");
    }
    lost = true;
  }
}

void fixTime() {
  d.Minute += 30;
  if (d.Minute > 59) {
    d.Hour += 1;
    d.Minute -= 60;
  }
  d.Hour += 5;
  if (d.Hour > 23) {
    d.Hour -= 24;
  }
}

void storeData() {
  GPS_satellite = String(d.satelliteCount);
  if (d.polarity <= 1) {
    i = 1.0;
    j = 1.0;
  } else if (d.polarity == 2) {
    i = 1.0;
    j = -1.0;
  } else if (d.polarity == 3) {
    i = -1.0;
    j = 1.0;
  } else if (d.polarity = 4) {
    i = -1.0;
    j = -1.0;
  } else {
    i = 1.0;
    j = 1.0;
  }
  GPS_latitude = String((i * (d.latitude / latitudeFactor)), 8);
  GPS_longitude = String((j * (d.longitude / longitudeFactor)), 7);
  GPS_distance = String(((d.distQuotient * dataDivisionFactor) + d.distRemainder));
  GPS_altitude = String(((float)(((d.altQuotient * dataDivisionFactor) + d.altRemainder) / 100.0)), 2);
  fixTime();
  if (d.satelliteCount > 0) {

    if (d.Hour < 9) GPS_time = "0" + String(d.Hour);
    else GPS_time = String(d.Hour);
    GPS_time += ":";
    if (d.Minute < 9) GPS_time += "0" + String(d.Minute);
    else GPS_time += String(d.Minute);
  } else GPS_time = "??:??";
  GPS_speed = String(d.velocity);
  BME_temp = String((((float)((d.tempQuotient * dataDivisionFactor) + d.tempRemainder)) / 100.0), 2);
  BME_altitude = String((((d.altPolarity) ? 1.0 : -1.0) * ((float)((d.AltitudeQuotient * dataDivisionFactor) + d.AltitudeRemainder)) / 100.0), 2);
  BV = String(((((float)((d.bvQuotient * dataDivisionFactor) + d.bvRemainder)) / 100.0) * batConst), 2);
  Hdop = String(d.hdop);
}


void displayData() {
  Serial.print("Satellite Count: ");
  Serial.println(GPS_satellite);
  if (d.satelliteCount > 0) {
    Serial.print("Latitude: ");
    Serial.print(GPS_latitude);  // V0
    Serial.print("\tLongitude: ");
    Serial.println(GPS_longitude);  // V1

    Serial.print("Distance: ");
    Serial.print(GPS_distance);  // V2
    Serial.println(" metre");

    Serial.print("GPS Altitude: ");
    Serial.print(GPS_altitude);  // V3
    Serial.println(" meter");

    Serial.print("Time: ");
    Serial.println(GPS_time);

    Serial.print("Speed: ");
    Serial.print(GPS_speed);  // V4
    Serial.println(" km/hr");

    Serial.println("Hdop: "+Hdop);
  }
  Serial.print("Temperature: ");
  Serial.println(BME_temp);  // V5

  Serial.print("Home lock: ");
  Serial.println((int)(d.homeLock));

  Serial.println("Battery voltage: " + BV);

  Serial.print("BME Altitude: ");
  Serial.print(BME_altitude);  // V6
  Serial.println(" meter \n\n");
}

void updateLCD() {
  lcd1.clear();

  lcd1.setCursor(0, 0);
  if (Mode && !sa) lcd1.write(4);        // wifi
  else if (!Mode && !sa) lcd1.write(3);  // hotspot
  else if (!Mode && sa) lcd1.write(5);   // standAlone
  else lcd1.print("!");

  lcd1.setCursor(2, 0);
  if (d.homeLock != 0) lcd1.write(2);  // locked
  else lcd1.write(1);                    // unlocked

  lcd1.setCursor(4, 0);
  lcd1.print("Telemetry");

  lcd1.setCursor(15, 0);
  lcd1.print(GPS_time);

  lcd1.setCursor(0, 1);
  lcd1.print("Sat " + GPS_satellite);

  lcd1.setCursor(7, 1);
  lcd1.print("Q "+Hdop+"%");

  lcd1.setCursor(16, 1);
  t = (int)(((float)((d.tempQuotient * dataDivisionFactor) + d.tempRemainder)) / 100.0);
  lcd1.print(t);
  lcd1.write(0);
  lcd1.print("C");

  lcd1.setCursor(0, 2);
  lcd1.print("Sp " + ((d.satelliteCount > 0) ? GPS_speed : "?"));
  lcd1.print("km/h");

  lcd1.setCursor(11, 2);
  lcd1.print("Dis " + ((d.satelliteCount > 0) ? GPS_distance : "?"));
  lcd1.print("m");

  lcd1.setCursor(0, 3);
  lcd1.print("Alt ");
  t = ((int)(((d.altPolarity) ? -1.0 : 1.0) * ((float)((d.AltitudeQuotient * dataDivisionFactor) + d.AltitudeRemainder)) / 100.0));
  lcd1.print(t);
  lcd1.print("m");

  lcd1.setCursor(14, 3);
  lcd1.print(BV + "v");

  lcd2.clear();
  lcd2.setCursor(0, 0);
  lcd2.print("Lat ");
  lcd2.print(((d.satelliteCount > 0) ? (GPS_latitude + "0") : "?"));
  lcd2.setCursor(0, 1);
  lcd2.print("Lng ");
  lcd2.print(((d.satelliteCount > 0) ? (GPS_longitude + "00") : "?"));
}


void send_Blynk() {
  Blynk.virtualWrite(V0, GPS_latitude);
  Blynk.virtualWrite(V1, GPS_longitude);
  Blynk.virtualWrite(V2, ((d.distQuotient * dataDivisionFactor) + d.distRemainder));
  Blynk.virtualWrite(V3, ((float)(((d.altQuotient * dataDivisionFactor) + d.altRemainder) / 100.0)));
  Blynk.virtualWrite(V4, (int)(d.velocity));
  Blynk.virtualWrite(V5, (((float)((d.tempQuotient * dataDivisionFactor) + d.tempRemainder)) / 100.0));
  Blynk.virtualWrite(V6, (((d.altPolarity) ? -1.0 : 1.0) * ((float)((d.AltitudeQuotient * dataDivisionFactor) + d.AltitudeRemainder)) / 100.0));
  Blynk.virtualWrite(V7, (int)(d.satelliteCount));
  Blynk.virtualWrite(V8, BV);
  Blynk.virtualWrite(V9, (int)(d.hdop));
}
