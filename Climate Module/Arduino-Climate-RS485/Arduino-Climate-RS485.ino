/*
BME280 I2C Test.ino

Connecting the BME280 Sensor:
Sensor              ->  Board
-----------------------------
Vin (Voltage In)    ->  3.3V
Gnd (Ground)        ->  Gnd
SDA (Serial Data)   ->  A4 on Uno/Pro-Mini, 20 on Mega2560/Due, 2 Leonardo/Pro-Micro
SCK (Serial Clock)  ->  A5 on Uno/Pro-Mini, 21 on Mega2560/Due, 3 Leonardo/Pro-Micro

 */

/* ==== Includes ==== */
#include <BME280I2C.h>
#include <Wire.h>             // Needed for legacy versions of Arduino.
#include <Auto485.h>
#include <DHT.h>
/* ====  END Includes ==== */

/* ==== Defines ==== */
#define SERIAL_BAUD 9600
#define DE_PIN 2
#define RE_PIN 2 /* connect DE and RE on the MAX485 together */
Auto485 bus(DE_PIN, RE_PIN); // new Auto485 wrapper using DE_PIN & RE_PIN to toggle read/write mode on the MAX485
/* ==== END Defines ==== */

/* ==== Global Variables ==== */
BME280I2C bme;                   // Default : forced mode, standby time = 1000 ms
                              // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
bool metric = true;
/* ==== END Global Variables ==== */

int gas_sensor_one = 7;

int ldr_sensor1 = A0;
int ldr_sensor2 = A1;
int ldr_sensor3 = A3;

#define DHTPIN            5
#define DHTTYPE           DHT22
DHT dht(DHTPIN, DHTTYPE);

/* ==== Prototypes ==== */
/* === Print a message to stream with the temp, humidity and pressure. === */
void printBME280Data(Stream * client);
/* === Print a message to stream with the altitude, and dew point. === */
void printBME280CalculatedData(Stream* client);
/* ==== END Prototypes ==== */

/* ==== Setup ==== */
void setup() {
  bus.begin(9600);
  pinMode(gas_sensor_one,INPUT);
  //pinMode(gas_sensor_two,INPUT);
  pinMode(ldr_sensor1, INPUT);
  pinMode(ldr_sensor2, INPUT);
  pinMode(ldr_sensor3, INPUT);
  dht.begin();
  while(!Serial) {} // Wait
  while(!bme.begin()){
  Serial.println("Could not find BME280 sensor!");
  delay(1000);
  }
}
/* ==== END Setup ==== */

/* ==== Loop ==== */
void loop() {
   printBME280Data(&Serial);
   printBME280CalculatedData(&Serial);
   printGasOne(&Serial);
   printLdr1(&Serial);
   printLdr2(&Serial);
   printLdr3(&Serial);
   printDht(&Serial);
   delay(5000);
}
/* ==== End Loop ==== */

/* ==== Functions ==== */
void printBME280Data(Stream* client){
  bus.set_mode(Auto485::TX); // mode = transmit

  float temp(NAN), hum(NAN), pres(NAN);

   uint8_t pressureUnit(3);                                           // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi
   bme.read(pres, temp, hum, metric, pressureUnit);                   // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
  /* Alternatives to ReadData():
    float temp(bool celsius = false);
    float pres(uint8_t unit = 0);
    float hum();
  
    Keep in mind the temperature is used for humidity and
    pressure calculations. So it is more effcient to read
    temperature, humidity and pressure all together.
   */
  client->print("Temp:");
  client->print(temp,0);
  client->println();
  client->print("Humid:");
  client->print(hum,0);
  client->println();
  client->print("Pres:");
  client->print(pres);
  client->println();
  
  bus.set_mode(Auto485::RX); // mode = receive, will pause until all pending serial data has been transmitted
}

void printBME280CalculatedData(Stream* client){
  bus.set_mode(Auto485::TX); // mode = transmit
  int altitude = bme.alt(metric);
  float dewPoint = bme.dew(metric);
  client->print("Alt:");
  client->print(altitude);
  client->println();
  client->print("Dew:");
  client->print(dewPoint,0);
  client->println();
  bus.set_mode(Auto485::RX);
}

void printGasOne(Stream* client){
  bus.set_mode(Auto485::TX); // mode = transmit
  int gas_value;
  gas_value=analogRead(gas_sensor_one);
  client->print("Gas1:");
  client->print(gas_value);
  client->println();
  bus.set_mode(Auto485::RX);
}

void printLdr1(Stream* client){
  bus.set_mode(Auto485::TX); // mode = transmit
  int ldr_value;
  ldr_value=analogRead(ldr_sensor1);
  client->print("LDRone:");
  client->print(ldr_value);
  client->println();
  bus.set_mode(Auto485::RX);
}
void printLdr2(Stream* client){
  bus.set_mode(Auto485::TX); // mode = transmit
  int ldr_value;
  ldr_value=analogRead(ldr_sensor2);
  client->print("LDRtwo:");
  client->print(ldr_value);
  client->println("");
  bus.set_mode(Auto485::RX);
}
void printLdr3(Stream* client){
  bus.set_mode(Auto485::TX); // mode = transmit
  int ldr_value;
  ldr_value=analogRead(ldr_sensor3);
  client->print("LDRthree:");
  client->print(ldr_value);
  client->println("");
  bus.set_mode(Auto485::RX);
}

void printDht(Stream* client){
  bus.set_mode(Auto485::TX); // mode - transmit
  int h = dht.readHumidity();
  int t = dht.readTemperature();
  client->print("DHTh:");
  client->print(h);
  client->println();
  client->print("DHTt:");
  client->print(t);
  client->println("/n");
  
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
   // return;
  }
}

/* ==== END Functions ==== */
