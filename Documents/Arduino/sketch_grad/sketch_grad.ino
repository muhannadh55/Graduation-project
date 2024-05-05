
//MAX30100 HeartRate Sensor
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
PulseOximeter pox;
#define REPORTING_PERIOD_MS     1000
uint32_t tsLastReport = 0;

//DHT11 Humidity Sesnor
#include <DHT11.h>
DHT11 dht11(4);

//UltraSonic Sensor
#include <Ultrasonic.h>
Ultrasonic ultrasonic(12,14);
	
//Servo Motor
#include <ESP32Servo.h>
int pos = 0; 
Servo MyServo;

//IR sensor
int IRSensor = 5;
int LED = 13; 

//Function For BeatDetected
void onBeatDetected()
{
    Serial.println("Beat!");
}

void setup()
{
    Serial.begin(115200);
    Serial.print("Initializing pulse oximeter..");
    if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }
       pox.setOnBeatDetectedCallback(onBeatDetected);

   MyServo.attach(26);

  //IR sensor
  Serial.println("Serial Working"); // Test to check if serial is working or not
  pinMode(IRSensor, INPUT); // IR Sensor pin INPUT
  pinMode(LED, OUTPUT); // LED Pin Output
   
}

void loop()
{

//DHT11 Humidity Sensor
  int humidity = dht11.readHumidity();
   if (humidity != DHT11::ERROR_CHECKSUM && humidity != DHT11::ERROR_TIMEOUT)
    {
        Serial.print("Humidity: ");
        Serial.print(humidity);
        Serial.println(" %");
    }
    else
    {
        Serial.println(DHT11::getErrorString(humidity));
    }
    delay(1000);

//MAX30100 HeartRate Sensor
    pox.update();
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) 
    {
        Serial.print("Heart rate:");
        Serial.print(pox.getHeartRate());
       Serial.print("bpm / SpO2:");
        Serial.print(pox.getSpO2());
        Serial.println("%");
 tsLastReport = millis();
}

//Ultrasonic
Serial.print("UltraSonic Sensor: ");
  Serial.print(ultrasonic.read()); // Prints the distance on the default unit (centimeters)
  Serial.println("cm");


//servo motor
for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    MyServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    MyServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }

  //IR sensor
   int sensorStatus = digitalRead(IRSensor); // Set the GPIO as Input
  if (sensorStatus == 1) // Check if the pin high or not
  {
    // if the pin is high turn off the onboard Led
    digitalWrite(LED, LOW); // LED LOW
    Serial.println("Motion Ended!"); // print Motion Detected! on the serial monitor window
  }

  else
  {
    //else turn on the onboard LED
    digitalWrite(LED, HIGH); // LED High
    Serial.println("Motion Detected!"); // print Motion Ended! on the serial monitor window
  }
}