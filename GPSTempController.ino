/****************************************************************
 * MicroOLED_Clock.ino
 * Analog Clock demo using SFE_MicroOLED Library
 * Jim Lindblom @ SparkFun Electronics
 * Original Creation Date: October 27, 2014
 *
 * This sketch uses the MicroOLED library to draw a 3-D projected
 * cube, and rotate it along all three axes.
 *
 * Development environment specifics:
 *  Arduino 1.0.5
 *  Arduino Pro 3.3V
 *  Micro OLED Breakout v1.0
 *
 * This code is beerware; if you see me (or any other SparkFun employee) at the
 * local, and you've found our code helpful, please buy us a round!
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/
#include <Wire.h>  // Include Wire if you're using I2C
// #include <SPI.h>  // Include SPI if you're using SPI
#include "SFE_MicroOLED.h"  // Include the SFE_MicroOLED library
#include "DHT.h"
#include "Adafruit_MLX90614.h"
#include "TinyGPS++.h"
#include "blynk.h"

/////////////////////////////////////////////////////////////////////
// DHT22
#define DHTTYPE DHT22
const int DHTPin = D4;
DHT dht(DHTPin, DHTTYPE);
/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
// OLED Object
MicroOLED oled(D7, 0);  // I2C Example
/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
// MLX90614 Object
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
// TinyGPSPlus Object
TinyGPSPlus gps;
String msg5;
float spd;
float lat;
float lon;
float sats;
float hdop;
String bearing;
String slocation;
unsigned long lastSync; // Particle Time
unsigned long last_fix; // GPS Last Fix
unsigned long lastCell = 0; // Cell Last Read
unsigned long lastGPSPublish = 0; // GPS Published to cloud

//Fuel (Battery) Variables
FuelGauge fuel;
double SoC;
/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
// Blynk Definition
char auth[] = "4e6588c115b04c15afa5225a8a3e2d78"; //Blynk's Auth Token
#define BLYNK_HEARTBEAT 60
#define BLYNK_VIRTUAL_PIN_GPS_LOCATION     V0
#define BLYNK_VIRTUAL_PIN_GPS_SPEED        V1
#define BLYNK_VIRTUAL_PIN_GPS_BEARING      V2
#define BLYNK_VIRTUAL_PIN_GPS_SAT_NUMBER   V3
#define BLYNK_VIRTUAL_PIN_GPS_ACCURACY     V4
#define BLYNK_VIRTUAL_PIN_MESSAGE          V5
#define BLYNK_VIRTUAL_PIN_DHT_HUMIDITY     V6
#define BLYNK_VIRTUAL_PIN_DHT_TEMP         V7
#define BLYNK_VIRTUAL_PIN_MLX_AMBIENT_TEMP V8
#define BLYNK_VIRTUAL_PIN_MLX_OBJECT_TEMP  V9
#define BLYNK_VIRTUAL_PIN_PUSH_BUTTON      V10

BlynkTimer timer;

bool isFirstConnect = true;

/////////////////////////////////////////////////////////////////////
// Button Definition
// Set your LED and physical button pins here
const int ledPin = D7;
const int btnPin = D3;
int ledState = LOW;
int btnState = HIGH;

// When App button is pushed - switch the state
BLYNK_WRITE(BLYNK_VIRTUAL_PIN_PUSH_BUTTON) {
  ledState = param.asInt();
  digitalWrite(ledPin, ledState);
  if(ledState == 1) {
    displayInfoOnScreen("received!", 0, false, false);
    readMLXTemp();
  }
}

void setup() {
  /////////////////////////////////////////////////////////////////////
  // Oled Initialize
  oled.begin();     // Initialize the OLED
  oled.clear(PAGE); // Clear the display's internal memory
  oled.clear(ALL);  // Clear the library's display buffer
  oled.display();   // Display what's in the buffer (splashscreen)

  displayInfoOnScreen("Hi!", 1, true, false);
  /////////////////////////////////////////////////////////////////////
  Serial1.begin(9600);      // Tx/RX connection to gps
  Serial.begin(9600);

  // Setup TimeZone & lastSync time
  Time.zone(7);
	lastSync = Time.now();

  Particle.keepAlive(45);

  /////////////////////////////////////////////////////////////////////
  // blynk Initialize
  Blynk.begin(auth);
  /////////////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////////////
  // Button Setup
  pinMode(ledPin, OUTPUT);
  pinMode(btnPin, INPUT_PULLUP);
  digitalWrite(ledPin, ledState);

  while (Blynk.connect() == false) {
    displayInfoOnScreen("...connecting", 0, false, false);
  }

  dht.begin();
  mlx.begin();

  timer.setInterval(100L, checkPhysicalButton);
  timer.setInterval(5000L, readTempFromDHT);
  timer.setInterval(30000L, sendGPS);
}

void loop() {
  timer.run(); // Blynk Timer;
  /* sync the clock once a day on Particle Cloud */
	if (Time.now() > lastSync + 86400) {
		Particle.syncTime();
		lastSync = Time.now();
	}

  /* read battery state every 10 min */
  if (Time.now() > lastCell + 600) {
    SoC = fuel.getSoC();
    lastCell = Time.now();
  }

  // Blynk & GPS
  Blynk.run();
  // This sketch displays information every time a new sentence is correctly encoded.

  while (Serial1.available()) {
    char c = Serial1.read();
    gps.encode(c);
  }

  if (gps.location.isValid()) {
		last_fix = Time.now() - gps.location.age() / 1000;
	}

  if (millis() > 30000 && gps.charsProcessed() < 10) {
       msg5 = "No GPS detected";
       Blynk.virtualWrite(V5, msg5);
       while(true);
   }
}

/////////////////////////////////////////////////////////////////////
// Button Methods
void checkPhysicalButton()
{
  if (digitalRead(btnPin) == LOW) {
    // btnState is used to avoid sequential toggles
    if (btnState != LOW) {

      // Toggle LED state
      ledState = !ledState;
      digitalWrite(ledPin, ledState);

      // Update Button Widget
      Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_PUSH_BUTTON, ledState);
      readMLXTemp();
    }
    btnState = LOW;
  } else {
    btnState = HIGH;
    }
  }

//////////////////////////////////////////////////////////////////////////////////////////
// MLX Sensor Method
void readMLXTemp() {
  float mlxTemp = mlx.readObjectTempC();
  float mlxAmbientTemp = mlx.readAmbientTempC();

  Serial.print("Ambient = "); Serial.print(mlxAmbientTemp);
  Serial.print("*C\tObject = "); Serial.print(mlxTemp); Serial.println("*C");

  Serial.println();
  String mlxInfo = "Ambient: " + String(mlxAmbientTemp);
  String objectTempInfo = "Object: = " + String(mlxTemp);
  Particle.publish("MLX", mlxInfo);
  displayInfoOnScreen(String(mlx.readObjectTempC()), 3, false, false);
  Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_MLX_AMBIENT_TEMP, mlxAmbientTemp);
  Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_MLX_OBJECT_TEMP, mlxTemp);
}
//////////////////////////////////////////////////////////////////////////////////////////
// DHT Sensor Method
void readTempFromDHT() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
        return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.print(" *C ");
  Serial.print(hif);
  Serial.println(" *F");
  String dhtInfo = "Hmdty: " + String(h) + "% " + "Temp: " + String(t) + "*C";
  Particle.publish("DHT", dhtInfo);
  displayInfoOnScreen(Time.format(Time.now(), "%I:%M%p"), 0, false, true);
  Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_DHT_HUMIDITY, h);
  Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_DHT_TEMP, t);
}
  //////////////////////////////////////////////////////////////////////////////////////////
void displayInfoOnScreen(String title, int fontType, bool setCursorCenter, bool showDHT) {
  if(showDHT == false) {
    int middleX = oled.getLCDWidth() / 2;
    int middleY = oled.getLCDHeight() / 2;

    oled.clear(PAGE);
    oled.setFontType(fontType);
    if(setCursorCenter == false) {
      oled.setCursor(0,0);
    } else {
      oled.setCursor(middleX - (oled.getFontWidth() * (title.length()/2)),
                   middleY - (oled.getFontWidth() / 2));
    }
    oled.print(title);
    oled.display();
  } else {
    oled.clear(PAGE);            // Clear the display
    oled.setCursor(0, 0);        // Set cursor to top-left
    oled.setFontType(0);
    oled.print(title);
    oled.setCursor(0, 16);       // Set cursor to top-middle-left
    oled.setFontType(4);
    oled.print(dht.readTemperature());
    oled.setFontType(0);         // Repeat
    oled.print(" C");
    oled.setCursor(0, 32);
    oled.setFontType(4);
    oled.print(dht.readHumidity());
    oled.setFontType(0);
    oled.print(" \%");
    oled.display();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
// I2CScanner Methods
void scanI2CDevices() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}
/////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
// GPS Method
void sendGPS() {
   if (gps.location.isValid()) {
       lat = gps.location.lat();    //get latitude and longitude
       lon = gps.location.lng();
       Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_GPS_LOCATION, 1, lat, lon, "MED");    //write to Blynk map
       //create location string which can be used directly in Google maps
       slocation = String(lat) + "," + String(lon);
       spd = gps.speed.kmph();    //get speed
       Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_GPS_SPEED, spd);
       bearing = TinyGPSPlus::cardinal(gps.course.value()); //direction
       Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_GPS_BEARING, bearing);
       sats = gps.satellites.value();    //get number of satellites
       Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_GPS_SAT_NUMBER, sats);
       hdop = gps.hdop.value() / 10;    //get accuracy of location
       Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_GPS_ACCURACY, hdop);
       msg5 = "GPS Good";
       Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_MESSAGE, msg5);

       if(Time.now() > lastGPSPublish + 60) {
         lastGPSPublish = Time.now();
         Particle.publish("GPS: Med", slocation);
       }
   }
   else
   {
       msg5 = "Invalid Data";
       Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_MESSAGE, msg5);
   }
}
//////////////////////////////////////////////////////////////////////////////////////////
