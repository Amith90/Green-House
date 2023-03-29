#define BLYNK_TEMPLATE_ID "TMPLaovbQkLP"
#define BLYNK_DEVICE_NAME "Smart Green House Monitor"
#define BLYNK_AUTH_TOKEN "bppcw0hMZdY_2usTg9GeX60e5cevnytb"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include <DHT.h>
#include <NewPing.h>
#include <ezButton.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "SLT_FIBER_zaxz2";  // type your wifi name
char pass[] = "4S2ZPyRu";  // type your wifi password

BlynkTimer timer;

//Moisture sensor (Capacitive Soil Moisture Sensor)
int moist_sensorpin=A0;
int moist_sensorvalue=0;
int moist_outputvalue=0;

//Gas sensor(MQ-2 Gas Sensor)
int gas_sensorpin=A3;
int gas_sensorvalue=0;

// Temparature & Humidity Sensor (DHT22)
#define DHTPIN 13 //Connect Out pin to D2 in NODE MCU
#define DHTTYPE DHT22  
DHT dht(DHTPIN, DHTTYPE);

//ultrasonic distance sensor (HC-SRO5) 
#define TRIGGER_PIN 14  // Pin to trigger the HC-SR04
#define ECHO_PIN 12  // Pin to read the echo from the HC-SR04
#define MAX_DISTANCE 200  // Maximum distance to measure (in cm)
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);  // Create a NewPing object

// Motion Sensor
int ledPin = 27;  // pin for the LED
int motionPin = 2;  // pin for the PIR motion sensor

// Rain Sensor
int rainsensorPin = 35; // pin for the rain sensor  sensor
int rainsensorvalue = 0; // variable to store the rain sensor status

// Pressure sensor - BMP180
Adafruit_BMP085 bmp;

// Light Sensor - LM393 
int lightSensorPin = 34; 
int lightSensorvalue = 0;


LiquidCrystal_I2C lcd(0x27, 20, 4);  // specify I2C address and dimensions of LCD display

#define BUTTON_PIN_1 5  // ESP32 pin GIOP22 connected to button 1's pin
#define BUTTON_PIN_2 18  // ESP32 pin GIOP23 connected to button 2's pin
#define BUTTON_PIN_3 19  // ESP32 pin GIOP23 connected to button 3's pin
#define BUTTON_PIN_4 23  // ESP32 pin GIOP23 connected to button 4's pin

#define RELAY_PIN_1 26   // ESP32 pin GIOP27 connected to relay 1's pin
#define RELAY_PIN_2 25   // ESP32 pin GIOP26 connected to relay 2's pin
#define RELAY_PIN_3 33   // ESP32 pin GIOP26 connected to relay 3's pin
#define RELAY_PIN_4 32   // ESP32 pin GIOP26 connected to relay 4's pin

#define button1_vpin    V7
#define button2_vpin    V8
#define button3_vpin    V9  
#define button4_vpin    V10

ezButton button1(BUTTON_PIN_1);  // create ezButton object for button 1
ezButton button2(BUTTON_PIN_2);  // create ezButton object for button 2
ezButton button3(BUTTON_PIN_3);  // create ezButton object for button 3
ezButton button4(BUTTON_PIN_4);  // create ezButton object for button 4

// variables will change:
int relay1_state = LOW;  // the current state of relay 1
int relay2_state = LOW;  // the current state of relay 2
int relay3_state = LOW;  // the current state of relay 3
int relay4_state = LOW;  // the current state of relay 4

//------------------------------------------------------------------------------
// This function is called every time the device is connected to the Blynk.Cloud
// Request the latest state from the server
BLYNK_CONNECTED() {
  Blynk.syncVirtual(button1_vpin);
  Blynk.syncVirtual(button2_vpin);
  Blynk.syncVirtual(button3_vpin);
  Blynk.syncVirtual(button4_vpin);
}

//--------------------------------------------------------------------------
// This function is called every time the Virtual Pin state change
//i.e when web push switch from Blynk App or Web Dashboard
BLYNK_WRITE(button1_vpin) {
  relay1_state = param.asInt();
  digitalWrite(RELAY_PIN_1, relay1_state);
}
//--------------------------------------------------------------------------
BLYNK_WRITE(button2_vpin) {
  relay2_state = param.asInt();
  digitalWrite(RELAY_PIN_2, relay2_state);
}
//--------------------------------------------------------------------------
BLYNK_WRITE(button3_vpin) {
  relay3_state = param.asInt();
  digitalWrite(RELAY_PIN_3, relay3_state);
}
//--------------------------------------------------------------------------
BLYNK_WRITE(button4_vpin) {
  relay4_state = param.asInt();
  digitalWrite(RELAY_PIN_4, relay4_state);
}
//--------------------------------------------------------------------------

void sendSensor()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit
  int moist_sensorvalue=analogRead(moist_sensorpin);
  float moisture_percent = ((100*(moist_sensorvalue -4095) / (2550 - 4095)));
  int gas_sensorvalue=analogRead(gas_sensorpin);
  int distance = sonar.ping_cm();  // Measure the distance to the object
  int waterlevel = (120-distance);
  int rainsensorvalue = analogRead(rainsensorPin);
  float rainsensorpercentage = analogRead((100-(100*(rainsensorvalue-2000)/(4095-2000))));
  float t1 = bmp.readTemperature();
  float a = bmp.readAltitude();
  float p = bmp.readPressure();
  float sp = bmp.readSealevelPressure();
  int lightSensorvalue = analogRead(lightSensorPin);
  int light = (4095-lightSensorvalue);


  bool motionDetected = false;  // variable to store the motion sensor status

  int motionStatus = digitalRead(motionPin);  // read the motion sensor status

  if (motionStatus == 1) {   
        motionDetected = true;
  }
  else {
    motionDetected = false;
  }

  // If motion is detected, turn on the LED
  if (motionDetected) {
    digitalWrite(ledPin, HIGH);
    Blynk.virtualWrite(V6, motionStatus);
    Blynk.notify("Motion detected!");  // send a notification through Blynk
  } else {  // if no motion is detected
    digitalWrite(ledPin, LOW);  // turn off the LED
  }



  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1);
  }


  // You can send any value at any time.
  // Please don't send more that 10 values per second.
    Blynk.virtualWrite(V0, t);
    Blynk.virtualWrite(V1, h);
    Blynk.virtualWrite(V2, moist_sensorvalue);
    Blynk.virtualWrite(V3, gas_sensorvalue);
    Blynk.virtualWrite(V4, waterlevel);
    Blynk.virtualWrite(V5, moisture_percent);
    Blynk.virtualWrite(V11, rainsensorvalue);
    Blynk.virtualWrite(V12, t1);
    Blynk.virtualWrite(V13, a);
    Blynk.virtualWrite(V14, p);
    Blynk.virtualWrite(V15, light);


    Serial.print("Temperature : ");
    Serial.print(t);
    Serial.print("°C  |  ");
    Serial.print("Humidity : ");
    Serial.print(h);
    Serial.print("%");
    Serial.print("  |  Moisture : ");
    Serial.print(moist_sensorvalue);
    Serial.print(" ->  ");
    Serial.print(moisture_percent);
    Serial.print("%  |  Gas : ");
    Serial.print(gas_sensorvalue);  
    Serial.print("  |  ");
    Serial.print("Water Level : ");
    Serial.print(waterlevel); 
    Serial.print("cm  | Rain :  ");
    Serial.print(rainsensorvalue);
    Serial.print(" ->  ");
    Serial.print(rainsensorpercentage);
    Serial.print("% | Motion : "); 
    Serial.print(motionStatus);
    Serial.print(" | Pump :");  
    Serial.print(relay1_state);
    Serial.print(" | Fan :");  
    Serial.print(relay2_state);   
    Serial.print(" | Valve1 :");  
    Serial.print(relay3_state);   
    Serial.print(" | Valve2 :");  
    Serial.print(relay4_state);
    Serial.print(" | Tem1 :");  
    Serial.print(t1);
    Serial.print("°C  | Pressure :  ");   
    Serial.print(p);   
    Serial.print(" Pa | Altitude : ");  
    Serial.print(a);
    Serial.print(" m | Light : "); 
    Serial.println(lightSensorvalue);

    // Display values on the LCD
    lcd.setCursor(0, 0);
    lcd.print("TemIn:");
    lcd.print(t ,1);
    lcd.print("C");
    lcd.print("/Out:"); 
    lcd.print(t1); 
    lcd.setCursor(0, 1);
    lcd.print("Hum:");
    lcd.print(h,1);
    lcd.print("%");
    lcd.print("   Fog:");
    lcd.print(rainsensorpercentage,0);
    lcd.print("%");
    lcd.setCursor(0, 2);
    lcd.print("Moist:");
    String MoistString = String((int)moisture_percent);
    while (MoistString.length() < 3) {
    MoistString = "0" + MoistString;
    }
    lcd.print(MoistString);
    lcd.print("%");
    lcd.setCursor(11, 2);
    lcd.print("SunL:");
    int Lightvalue =(4095- lightSensorvalue);
    String LightString = String(Lightvalue);
    while (LightString.length() < 4) {
    LightString = "0" + LightString;
    }    
    lcd.print(LightString);
    lcd.setCursor(0, 3);
    lcd.print("Gas:");
    String GasString = String(gas_sensorvalue);
    while (GasString.length() < 4) {
    GasString = "0" + GasString;
    }
    lcd.print(GasString);
    lcd.setCursor(11, 3);
    lcd.print("WL:");
    String distanceString = String(waterlevel);
    while (distanceString.length() < 3) {
    distanceString = "0" + distanceString;
     }
    lcd.print(distanceString);
    lcd.print("cm");
    lcd.print(" ");
}

void setup()
{   
  Serial.begin(115200);  // initialize serial
  
  lcd.init();  // initialize LCD display
  lcd.backlight();  // turn on backlight
  pinMode(ledPin, OUTPUT);  // set the LED pin as output
  pinMode(motionPin, INPUT_PULLUP);  // set the motion sensor pin as input
  pinMode(rainsensorPin, INPUT);   // Configure the rain sensor pin
  Blynk.begin(auth, ssid, pass);
  dht.begin();
  timer.setInterval(1000L, sendSensor);
  bmp.begin();

  pinMode(RELAY_PIN_1, OUTPUT); // set ESP32 pin to output mode
  pinMode(RELAY_PIN_2, OUTPUT); // set ESP32 pin to output mode
  pinMode(RELAY_PIN_3, OUTPUT); // set ESP32 pin to output mode
  pinMode(RELAY_PIN_4, OUTPUT); // set ESP32 pin to output mode
  button1.setDebounceTime(50);  // set debounce time to 50 milliseconds
  button2.setDebounceTime(50);  // set debounce time to 50 milliseconds
  button3.setDebounceTime(50);  // set debounce time to 50 milliseconds
  button4.setDebounceTime(50);  // set debounce time to 50 milliseconds

  //During Starting all Relays should TURN OFF
  digitalWrite(RELAY_PIN_1, HIGH);
  digitalWrite(RELAY_PIN_2, HIGH);
  digitalWrite(RELAY_PIN_3, HIGH);
  digitalWrite(RELAY_PIN_4, HIGH);
 
  }

void loop() {
  Blynk.run();
  timer.run();

  button1.loop(); // MUST call the loop() function first
  button2.loop(); // MUST call the loop() function first
  button3.loop(); // MUST call the loop() function first
  button4.loop(); // MUST call the loop() function first

  if (button1.isPressed()) {
    Serial.println("Button 1 is pressed");

    // toggle state of relay 1
    relay1_state = !relay1_state;

    // control relay 1 according to the toggled state
    digitalWrite(RELAY_PIN_1, relay1_state);
    Blynk.virtualWrite(button1_vpin, relay1_state); //update button state
  }

  if (button2.isPressed()) {
    Serial.println("Button 2 is pressed");

    // toggle state of relay 2
    relay2_state = !relay2_state;

    // control relay 2 according to the toggled state
    digitalWrite(RELAY_PIN_2, relay2_state);
    Blynk.virtualWrite(button2_vpin, relay2_state); //update button state
  }

  if (button3.isPressed()) {
    Serial.println("Button 3 is pressed");

    // toggle state of relay 3
    relay3_state = !relay3_state;

    // control relay 3 according to the toggled state
    digitalWrite(RELAY_PIN_3, relay3_state);
    Blynk.virtualWrite(button3_vpin, relay3_state); //update button state   
  }

  if (button4.isPressed()) {
    Serial.println("Button 4 is pressed");

    // toggle state of relay 4
    relay4_state = !relay4_state;

    // control relay 4 according to the toggled state
    digitalWrite(RELAY_PIN_4, relay4_state);
    Blynk.virtualWrite(button4_vpin, relay4_state); //update button state
    
  }

}


