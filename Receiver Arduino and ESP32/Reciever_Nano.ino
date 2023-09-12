// RECIEVER ESP32

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define baudRate 9600
#define latitudeFactor 100000000.0
#define longitudeFactor 10000000.0
#define dataDivisionFactor 100

RF24 radio(4, 5); // 4->CE, 5->CSN

const byte address[6] = "00001";

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
typedef struct Data_Package {

  unsigned long latitude;           // 4 byte
  unsigned long longitude;          // 4 byte
  byte satelliteCount;              // 1 byte
  byte polarity;                    // 1 byte      // 1-> +lat,+long;  2-> +lat,-long;  3-> -lat,+long;  4-> -lat,-long
  byte distQuotient;                // 1 byte
  byte distRemainder;               // 1 byte
  byte altQuotient;                 // 1 byte
  byte altRemainder;                // 1 byte
  byte velocity;                    // 1 byte
  byte Hour;                        // 1 byte
  byte Minute;                      // 1 byte
  byte tempQuotient;                // 1 byte
  byte tempRemainder;               // 1 byte
  byte altPolarity;                 // 1 byte
  byte AltitudeQuotient;            // 1 byte
  byte AltitudeRemainder;           // 1 byte
                             // Total 22 bytes
} Data;
Data d; // Create a variable with the above structure


void setup() {
  Serial.begin(baudRate);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();
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

float i, j;
String GPS_latitude, GPS_longitude, GPS_satellite, GPS_distance, GPS_altitude, GPS_time, GPS_speed, BME_temp, BME_altitude;

void loop() {
  // Check whether there is data to be received
  if (radio.available()) {
    while (radio.available()) radio.read(&d, sizeof(Data)); // Read the whole data and store it into the 'data' structure

    storeData();
    displayData();

  }
}


void storeData() {
  GPS_satellite = String(d.satelliteCount);
  if (d.polarity <= 1) {
    i = 1.0;
    j = 1.0;
  }
  else if (d.polarity == 2) {
    i = 1.0;
    j = -1.0;
  }
  else if (d.polarity == 3) {
    i = -1.0;
    j = 1.0;
  }
  else if (d.polarity = 4) {
    i = -1.0;
    j = -1.0;
  }
  else {
    i = 1.0;
    j = 1.0;
  }
  GPS_latitude = String((i * (d.latitude / latitudeFactor)), 8);
  GPS_longitude = String((j * (d.longitude / longitudeFactor)), 7);
  GPS_distance = String(((d.distQuotient * dataDivisionFactor) + d.distRemainder));
  GPS_altitude = String(( (float)(((d.altQuotient * dataDivisionFactor) + d.altRemainder) / 100.0) ), 2);
  fixTime();
  if (d.satelliteCount > 0) GPS_time = String(d.Hour) + ":" + String(d.Minute);
  else GPS_time = " : ";
  GPS_speed = String(d.velocity);
  BME_temp = String((((float)((d.tempQuotient * dataDivisionFactor) + d.tempRemainder)) / 100.0), 2);
  BME_altitude = String(( ((d.altPolarity) ? 1.0 : -1.0) * ((float)((d.AltitudeQuotient * dataDivisionFactor) + d.AltitudeRemainder)) / 100.0), 2);
}


void displayData() {
  Serial.print("Satellite Count: ");
  Serial.println(GPS_satellite);
  if (d.satelliteCount > 0) {
    Serial.print("Latitude: ");
    Serial.print(GPS_latitude);
    Serial.print("\tLongitude: ");
    Serial.println(GPS_longitude);

    Serial.print("Distance: ");
    Serial.print(GPS_distance);
    Serial.println(" metre");

    Serial.print("GPS Altitude: ");
    Serial.print(GPS_altitude);
    Serial.println(" meter");

    Serial.print("Time: ");
    Serial.println(GPS_time);

    Serial.print("Speed: ");
    Serial.print(GPS_speed);
    Serial.println(" km/hr");
  }
  Serial.print("Temperature: ");
  Serial.println(BME_temp);

  Serial.print("BME Altitude: ");
  Serial.print(BME_altitude);
  Serial.println(" meter \n\n");

}
