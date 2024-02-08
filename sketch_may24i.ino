#include <DHT.h>                //DHT Library
#include <LiquidCrystal_I2C.h>  //Libraries
#include <Wire.h>
#include <OneWire.h>  //DS18B20 Library
#include <DallasTemperature.h>

#include <SoftwareSerial.h>
#define RX 10
#define TX 11
String AP = "ZN";                 // AP NAME
String PASS = "zanar123";         // AP PASSWORD
String API = "AXG6EOLR9UR9D9LF";  // Write API KEY
String HOST = "api.thingspeak.com";
String PORT = "80";
String field = "field1";
int countTrueCommand;
int countTimeCommand;
boolean found = false;
int valSensor = 1;
SoftwareSerial esp8266(RX, TX);


#define DHTPIN 8
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);


// Data wire is plugged into digital pin 9 on the Arduino
#define ONE_WIRE_BUS 9



LiquidCrystal_I2C lcd(0x27, 20, 4);  //LCD Display Configuration


const int waterSensorPin = A0;
const int moistSensorPin = A2;
const int water_level_relayPin = 2;
const int moist_level_relayPin = 3;
const int tempSens_relayPin = 4;

const int ldrPin = A1;        // LDR sensor connected to analog pin A1
const int firstMotorPin1 = 5;  // First Motor control pin connected to digital pin 2
const int firstMotorPin2 = 6;
const int motorEnable = 7;           // Second Motor control pin connected to digital pin 3
const int onThreshold = 400;         // Threshold value to turn on the Motor
const int offThreshold = 100;        // Threshold value to turn off the Motor
const int firstMotorOnTime = 3000;   // Time (in milliseconds) to keep the first Motor on
const int secondMotorOnTime = 3000;  // Time (in milliseconds) to keep the second Motor on

bool isFirstMotorOn = false;  // Track the status of the first Motor


const int waterThreshold = 300;
const int moistDry = 645;  // value for dry sensor
const int moistWet = 320;  // value for wet sensor

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);


bool tempconditionMet = false;
unsigned long startTime = 0;
const unsigned long relayDuration = 120000;  // 2 minutes in milliseconds



void setup() {

  Serial.begin(9600);
  esp8266.begin(115200);
  sendCommand("AT", 5, "OK");
  sendCommand("AT+CWMODE=1", 5, "OK");
  sendCommand("AT+CWJAP=\"" + AP + "\",\"" + PASS + "\"", 20, "OK");
  dht.begin();

  pinMode(waterSensorPin, INPUT);

  sensors.begin();  // Start up the library
  pinMode(water_level_relayPin, OUTPUT);
  digitalWrite(water_level_relayPin, LOW);

  pinMode(moist_level_relayPin, OUTPUT);
  digitalWrite(moist_level_relayPin, HIGH);

  pinMode(tempSens_relayPin, OUTPUT);
  digitalWrite(tempSens_relayPin, HIGH);



  pinMode(firstMotorPin1, OUTPUT);  // Set first Motor pin as output
  pinMode(firstMotorPin2, OUTPUT);
  pinMode(motorEnable, OUTPUT);        // Set second Motor pin as output
  digitalWrite(firstMotorPin1, HIGH);   // Initially turn off the first Motor
  digitalWrite(firstMotorPin2, HIGH);  // Initially turn off the second Motor
  Serial.begin(9600);                  // Initialize serial communication (for debugging, if needed)
  analogWrite(motorEnable, 130);

  lcd.init();
  lcd.backlight();
  lcd.begin(20, 4);
  lcd.setCursor(1, 0);
  lcd.print("Rooftop Greenhouse");
  lcd.setCursor(7, 1);
  lcd.print("System");
  delay(1000);
  lcd.clear();
}

void loop() {

  valSensor = getSensorData();
  String getData = "GET /update?api_key=" + API + "&" + field + "=" + String(valSensor);
  sendCommand("AT+CIPMUX=1", 5, "OK");
  sendCommand("AT+CIPSTART=0,\"TCP\",\"" + HOST + "\"," + PORT, 15, "OK");
  sendCommand("AT+CIPSEND=0," + String(getData.length() + 4), 4, ">");
  esp8266.println(getData);

  countTrueCommand++;
  sendCommand("AT+CIPCLOSE=0", 5, "OK");





  int waterLevel = analogRead(waterSensorPin);

  static unsigned long lastOffTime = 0;
  unsigned long currentTime = millis();
  unsigned long delayTime = 60000;  // Delay in milliseconds (e.g., 60000 = 1 minute)

  if (waterLevel > waterThreshold) {
    digitalWrite(water_level_relayPin, HIGH);
    lastOffTime = currentTime;  // Update the lastOffTime variable
  } else {
    // Check if enough time has passed since the relay was turned off
    if (digitalRead(water_level_relayPin) == HIGH && currentTime - lastOffTime >= delayTime) {
      digitalWrite(water_level_relayPin, LOW);
    }
  }
  /*Serial.print("water level: ");
  Serial.println(waterLevel); */

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();


  int moistsensorVal = analogRead(moistSensorPin);
  int percentageMoist = map(moistsensorVal, moistWet, moistDry, 100, 0);

  if (percentageMoist < 20) {

    digitalWrite(moist_level_relayPin, LOW);
  }
  if (percentageMoist >= 80) {
    digitalWrite(moist_level_relayPin, HIGH);
  }

  int ldrValue = analogRead(ldrPin);  // Read the LDR sensor value

  if (ldrValue > onThreshold && !isFirstMotorOn)

  {
    digitalWrite(firstMotorPin1, LOW);   // Turn on the first Motor
    delay(firstMotorOnTime);            // Keep the first Motor on for the specified time
    digitalWrite(firstMotorPin1, HIGH);  // Turn off the first Motor
    isFirstMotorOn = true;              // Set the flag to indicate that the first Motor is turned on
  }

  if (ldrValue < offThreshold && isFirstMotorOn)

  {
    digitalWrite(firstMotorPin2, LOW);   // Turn on the second Motor
    delay(secondMotorOnTime);            // Keep the second Motor on for the specified time
    digitalWrite(firstMotorPin2, HIGH);  // Turn off the second Motor
    isFirstMotorOn = false;              // Reset the flag since the second Motor is turned on
  }


  //Serial.println(ldrValue);


  // Send the command to get temperatures
  sensors.requestTemperatures();

  // Read the temperature in Celsius
  float Floor_temperatureValue = sensors.getTempCByIndex(0);

  // Print the temperature value
  /*Serial.println("Temperature: ");
  Serial.print(Floor_temperatureValue);
  Serial.print((char)176);  // Show degrees character
  Serial.println("C  |  "); */

  if (Floor_temperatureValue > 35)  // 35 needs to be changed.
  {
    // Check if the relay needs to be turned on
    if (startTime == 0) {
      // Turn on the relay
      digitalWrite(tempSens_relayPin, LOW);
      startTime = millis();  // Start the timer
    }

    // Check if the relay duration has passed
    else if (millis() - startTime >= relayDuration) {
      // Turn off the relay
      digitalWrite(tempSens_relayPin, HIGH);

      // Reset the condition and start time
      tempconditionMet = false;
      startTime = 0;
    }
  }

  lcd.setCursor(0, 0);
  lcd.print("Humidity ");
  lcd.print(humidity);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("Temperature ");
  lcd.print(temperature);
  lcd.print((char)223);
  lcd.print("C");

  lcd.setCursor(0, 2);
  lcd.print("LDR Value ");
  lcd.print(ldrValue);

  lcd.setCursor(0, 3);
  lcd.print("Moisture ");
  lcd.print(percentageMoist);
  lcd.print("%");
  /*
  lcd.setCursor(3, 8);
  lcd.print("T");
  lcd.print(Floor_temperatureValue);


  //lcd.setCursor(0, 1);
  //lcd.print("Soil_Moist:");
  // lcd.print(percentageMoist);
*/


  delay(1000);
}
int getSensorData() {
  float humidity = dht.readHumidity();
  return humidity;
}


void sendCommand(String command, int maxTime, char readReplay[]) {
  Serial.print(countTrueCommand);
  Serial.print(". at command => ");
  Serial.print(command);
  Serial.print(" ");
  while (countTimeCommand < (maxTime * 1)) {
    esp8266.println(command);      //at+cipsend
    if (esp8266.find(readReplay))  //ok
    {
      found = true;
      break;
    }

    countTimeCommand++;
  }

  if (found == true) {
    Serial.println("OYI");
    countTrueCommand++;
    countTimeCommand = 0;
  }

  if (found == false) {
    Serial.println("Fail");
    countTrueCommand = 0;
    countTimeCommand = 0;
  }

  found = false;
}
