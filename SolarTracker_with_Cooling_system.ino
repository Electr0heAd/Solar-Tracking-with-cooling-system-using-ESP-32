#include <ESP32Servo.h>
#include <OneWire.h>  // include OneWire library for DS18B20 sensor
#include <DallasTemperature.h> // include DallasTemperature library for DS18B20 sensor
#include <Wire.h> // include Wire library for ADXL345 sensor
#include <Adafruit_Sensor.h> // include Adafruit_Sensor library for MPU6050 sensor
#include <Adafruit_ADXL345_U.h> // include Adafruit_MPU6050_U library for MPU6050 sensor

// Define the pins used for each sensor
#define ONE_WIRE_BUS 17 // Data pin for DS18B20
#define ADXL345_SDA 21 // Serial Data pin for ADXL345
#define ADXL345_SCL 22 // Serial Clock pin for ADXL345
#define GPIO27 27 // Output pin for positive gravity signal
#define GPIO14 14 // Output pin for negative gravity signal

// Initialize the libraries and sensors
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);

Servo servoX;
Servo servoY;

int ldrPin1 = 39; // connect LDR1 to pin 32
int ldrPin2 = 36; // connect LDR2 to pin 33
int ldrPin3 = 34; // connect LDR3 to pin 34
int ldrPin4 = 35; // connect LDR4 to pin 35
int tm=0;
void setup() {
  //  pinMode(ldrPin1, INPUT);
  //  pinMode(ldrPin2, INPUT);
  //  pinMode(ldrPin3, INPUT);
  //  pinMode(ldrPin4, INPUT);
  Serial.begin(115200); // Start serial communication
  pinMode(GPIO27, OUTPUT); // Set GPIO27 as output
  pinMode(GPIO14, OUTPUT); // Set GPIO14 as output
  ds18b20.begin(); // Start DS18B20 sensor
 Wire.begin(ADXL345_SDA, ADXL345_SCL); // Start I2C communication for ADXL345 sensor
  adxl.begin(); // Start ADXL345 sensor
  servoX.attach(26); // attach servoX to pin 4
  servoY.attach(25); // attach servoY to pin 5
  servoX.write(90); // set servoX to initial position
  servoY.write(90); // set servoY to initial position
}

void loop() {
  int ldrValue1 = analogRead(ldrPin1);
  int ldrValue2 = analogRead(ldrPin2);
  int ldrValue3 = analogRead(ldrPin3);
  int ldrValue4 = analogRead(ldrPin4);
  Serial.print(ldrValue1);
  Serial.print(",");
  Serial.print(ldrValue2);
  Serial.print(",");
  Serial.print(ldrValue3);
  Serial.print(",");
  Serial.println(ldrValue4);
  float angleX = servoX.read();
  float angleY = servoY.read();

  if (ldrValue2 > ldrValue4 && ldrValue2 - ldrValue4 > 2) {
      servoX.write(angleX - 5);
  }
  else if (ldrValue2 < ldrValue4 && ldrValue4 - ldrValue2 > 2) {
    servoX.write(angleX + 5);
  }
  if (ldrValue1 > ldrValue3 && ldrValue1 - ldrValue3 > 2) {
      servoY.write(angleY - 5);
  }
  else if (ldrValue2 < ldrValue4 && ldrValue4 - ldrValue2 > 2) {
     servoY.write(angleY + 5);
  }
  tm++;
  if(tm==1){
    tm=0;
  ds18b20.requestTemperatures(); // Get the temperature value from DS18B20 sensor
  float tempC = ds18b20.getTempCByIndex(0); // Store the temperature value in Celsius
  Serial.print("temp = ");
  Serial.println(tempC);
  sensors_event_t event; // Create an event object for ADXL345 sensor
  adxl.getEvent(&event); // Get the event data from ADXL345 sensor
  float xGravity = event.acceleration.x; // Store the X-axis gravity value
  Serial.print("X axis= ");
  Serial.println(xGravity);
  if (tempC > 25) { // Check if temperature is greater than 25 degrees Celsius
    if (xGravity >= 15.4 && xGravity <=19.88) { // Check if X-axis gravity is greater than or equal to 0
      digitalWrite(GPIO27, HIGH); // Send positive gravity signal
      digitalWrite(GPIO14, LOW); // Turn off negative gravity signal
    } else if (xGravity <= 13.4 && xGravity >=8.70) { // Check if X-axis gravity is less than 0 and greater than -10
        digitalWrite(GPIO14, HIGH); // Send negative gravity signal
      digitalWrite(GPIO27, LOW); // Turn off positive gravity signal
    }
    else { // If temperature is less than or equal to 25 degrees Celsius
      digitalWrite(GPIO27, LOW); // Turn off positive gravity signal
      digitalWrite(GPIO14, LOW); // Turn off negative gravity signal
    }
  }
  else { // If temperature is less than or equal to 25 degrees Celsius
    digitalWrite(GPIO27, LOW); // Turn off positive gravity signal
    digitalWrite(GPIO14, LOW); // Turn off negative gravity signal
  }
  }
  delay(1400);
  
}
