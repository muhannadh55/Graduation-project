#include <WiFi.h>
#include <ESPAsyncWebServer.h>

#define ENA   22          // Enable/speed motors Right        GPIO14(D5)
#define ENB   23          // Enable/speed motors Left         GPIO12(D6)
#define IN_1  16          // L298N in1 motors Right           GPIO15(D8)
#define IN_2  17          // L298N in2 motors Right           GPIO13(D7)
#define IN_3  18           // L298N in3 motors Left            GPIO2(D4)
#define IN_4  19           // L298N in4 motors Left            GPIO0(D3)


//MAX30100 HeartRate Sensor
/*#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
PulseOximeter pox;
#define REPORTING_PERIOD_MS     1000
uint32_t tsLastReport = 0;
*/
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
/*void onBeatDetected()
{
    Serial.println("Beat!");
}
*/


String command;             //String to store app command state.
int speedCar = 800;         // 400 - 1023.
int speed_Coeff = 3;

const char* ssid = "maro";
const char* password = "marwanh#76";

AsyncWebServer server(80);

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);

  Serial.begin(115200);

  // Connecting to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  Serial.println(WiFi.SSID());
  delay(1000);
  server.begin(); /* Start the HTTP web Server */
  Serial.print("Connect to IP Address: ");
  Serial.print("http://");
  Serial.println(WiFi.localIP());
  
  // Starting Async Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    HTTP_handleRoot(request);
  });
  server.onNotFound([](AsyncWebServerRequest *request){
    HTTP_handleRoot(request);
  });
  server.begin();


  /*  Serial.print("Initializing pulse oximeter..");
    if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }
       pox.setOnBeatDetectedCallback(onBeatDetected);
*/
   MyServo.attach(26);

  //IR sensor
  Serial.println("Serial Working"); // Test to check if serial is working or not
  pinMode(IRSensor, INPUT); // IR Sensor pin INPUT
  pinMode(LED, OUTPUT); // LED Pin Output
  
}

void goAhead() {
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
  analogWrite(ENA, speedCar);

  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
  analogWrite(ENB, speedCar);
}

void goBack() {
  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);
  analogWrite(ENA, speedCar);

  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);
  analogWrite(ENB, speedCar);
}

void goRight(){ 

      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);
      analogWrite(ENA, speedCar);

      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);
      analogWrite(ENB, speedCar);
  }

void goLeft(){

      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);
      analogWrite(ENA, speedCar);

      digitalWrite(IN_3, HIGH);
      digitalWrite(IN_4, LOW);
      analogWrite(ENB, speedCar);
  }

void goAheadRight(){
      
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);
      analogWrite(ENA, speedCar/speed_Coeff);
 
      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);
      analogWrite(ENB, speedCar);
   }

void goAheadLeft(){
      
      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);
      analogWrite(ENA, speedCar);

      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);
      analogWrite(ENB, speedCar/speed_Coeff);
  }

void goBackRight(){ 

      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);
      analogWrite(ENA, speedCar/speed_Coeff);

      digitalWrite(IN_3, HIGH);
      digitalWrite(IN_4, LOW);
      analogWrite(ENB, speedCar);
  }

void goBackLeft(){ 

      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);
      analogWrite(ENA, speedCar);

      digitalWrite(IN_3, HIGH);
      digitalWrite(IN_4, LOW);
      analogWrite(ENB, speedCar/speed_Coeff);
  }


void stopRobot() {
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, LOW);
  analogWrite(ENA, speedCar);

  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, LOW);
  analogWrite(ENB, speedCar);
}

void loop() {
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
  /*  pox.update();
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) 
    {
        Serial.print("Heart rate:");
        Serial.print(pox.getHeartRate());
       Serial.print("bpm / SpO2:");
        Serial.print(pox.getSpO2());
        Serial.println("%");
 tsLastReport = millis();
}
*/
 // Ultrasonic Sensor
  int distance_cm = ultrasonic.read(); // Read distance in centimeters
  Serial.print("UltraSonic Sensor: ");
  Serial.print(distance_cm);
  Serial.println("cm");

  // Check if distance is less than or equal to 5cm
  if (distance_cm <= 5) {
    // Stop the car
    stopRobot();

    // Determine direction based on servo position
    if (pos < 90) {
      // Turn right
      for (pos = 0; pos <= 90; pos += 1) {
        MyServo.write(pos);
        delay(15);
      }
    } else {
      // Turn left
      for (pos = 180; pos >= 90; pos -= 1) {
        MyServo.write(pos);
        delay(15);
      }
    }
  } else {
    // If distance is greater than 5cm, continue moving ahead
    goAhead();
  }

  // Delay for stability
  delay(100);

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

void HTTP_handleRoot(AsyncWebServerRequest *request) {
  command = request->arg("State");
  if (command == "F") goAhead();
  else if (command == "B") goBack();
 else if (command == "L") goLeft();
      else if (command == "R") goRight();
      else if (command == "I") goAheadRight();
      else if (command == "G") goAheadLeft();
      else if (command == "J") goBackRight();
      else if (command == "H") goBackLeft();
      else if (command == "0") speedCar = 400;
      else if (command == "1") speedCar = 470;
      else if (command == "2") speedCar = 540;
      else if (command == "3") speedCar = 610;
      else if (command == "4") speedCar = 680;
      else if (command == "5") speedCar = 750;
      else if (command == "6") speedCar = 820;
      else if (command == "7") speedCar = 890;
      else if (command == "8") speedCar = 960;
      else if (command == "9") speedCar = 1023;
  else if (command == "S") stopRobot();

  request->send(200, "text/html", "");
}