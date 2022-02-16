//https://store.steampowered.com/app/1718240/Beaver_Clicker/
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <math.h>
//#include <NMEAGPS.h>

#define delTime 250

char buff[50];
int fileNumber;
int statLED = 13;
int resetOpenLog = 2;

String input;

using namespace std;

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

float temp;
float pressure;

float lat, lng, alt;
float mpxAlt;

float seconds = 0;

void setup() {
  Serial.begin(9600);
  ss.begin(9600);
  Serial.println();
  Serial.println("Begin");

  //InitSD();

  //InitBMP();

  Serial.println("bmpTemperature, lmTemperature, ntcTemperature, bmpPressure, mpxPressure, bmpAltitude, latitude, longitude, altitude, time");
}

void InitSD() {
  pinMode(statLED, OUTPUT);
  pinMode(resetOpenLog, OUTPUT);
 
  randomSeed(analogRead(0));
  //Reset OpenLog
  digitalWrite(resetOpenLog, LOW);
  delay(100);
  digitalWrite(resetOpenLog, HIGH);
  //Wait for OpenLog to respond with '<' to indicate it is alive and recording to a file
  while(1) {
    if(Serial.available())
    if(Serial.read() == '<') break;
  }  

  Serial.write(26);
  Serial.write(26);
  Serial.write(26);

  while(1) {
    if(Serial.available())
    if(Serial.read() == '>') break;
  }
 
  fileNumber = random(999);
  
  sprintf(buff, "new nate%03d.txt\r", fileNumber);
 
  Serial.print(buff);

  while(1) {
  if(Serial.available())
  if(Serial.read() == '>') break;
  }
  sprintf(buff, "append nate%03d.txt\r", fileNumber);
  Serial.print(buff);
  
  //Wait for OpenLog to indicate file is open and ready for writing
  while(1) {
  if(Serial.available())
  if(Serial.read() == '<') break;
  }
}
void InitBMP() {
  Serial.print("1");
  unsigned status;
  Serial.print("2");
  Serial.print("4");
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
    Serial.print("3");
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();
}

void loop() {
  ReadBMP();
  ReadGPS();

  float d[] = {
    temp,
    ReadLM(0, 100), //offset, sensitivity
    ReadNTC(34, -20), //off, grad
    pressure,
    ReadMPX(998.8f),
    BMPAltutude(998.8f),
    mpxAlt,
    lat,
    lng,
    alt,
    SecFromStart(delTime)
    //SatelitesInUse()
  };

  //CheckForInput();
  OutputData(d, 11); //Array of all the data, and length of array

  delay(delTime);

  while (ss.available() > 0)
    gps.encode(ss.read());
}

#pragma region readSensors
void ReadBMP() {
  float f = bmp.readTemperature();
  
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  temp = temp_event.temperature;
  pressure = pressure_event.pressure;
}
float BMPAltutude(float seaPressure){
  return bmp.readAltitude(seaPressure);
}

float ReadMPX(float seaPressure) {
  float volt = BitToVolt(1);
  float pressure = 10*(volt/(0.009*5)+(0.095/0.009));

  mpxAlt = 14366.45*(1-pow((seaPressure/1013.25), 0.190284f));
  return pressure;
}

float ReadLM(float lmOffset, float lmSens){
  float volt = BitToVolt(2);
  //return lmSens*volt+lmOffset;
  return volt;
}

float ReadNTC(float ntcOff, float ntcGrad) {
  float bit = analogRead(A0);
  float volt = BitToVolt(0);
  //return ntcOff + ntcGrad * volt;
  return volt;
}

void ReadGPS() {
  while (ss.available() > 0)
    gps.encode(ss.read());

  lat = gps.location.lat();
  lng = gps.location.lng();
  alt = gps.altitude.meters();
}
#pragma endregion readSensors

void OutputData(float data[], int lenght) {
  Serial.println();
  Serial.print("SHH ");

  for(int i = 0; i < lenght; i++)
  {
    if(i == 7 || i == 8) Serial.print(data[i], 6);
    else Serial.print(data[i]);
    if(i != lenght-1) Serial.print(", ");
  }
}

float BitToVolt(int n){    //Function to convert raw ADC-data (0-255) to volt
  int raw = analogRead(n);
  float volt = (float)raw*5.000/1023;
  return volt;
}
float SecFromStart(float time){
  seconds += time/1000;
  return seconds;
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis(); // Hent antall millisekunder siden reset av Arduino-kortet
  do // Testen gjøres etter loopen er kjørt
  {
    while (ss.available())gps.encode(ss.read());
  }
  while (millis() - start < ms); // Testen gjøres før loopen er kjørt
}
