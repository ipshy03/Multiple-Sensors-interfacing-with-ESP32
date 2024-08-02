// Author: Ipsita Mohanty

//---->IMPORTANT NOT TO RUN THE CODE<--------
//Change the pin as per the connection/requirement.
//Download the library for the library manager and given link.
//Uncomment the Sensor Function in void loop section 


/*
---TYPES OF SENSOR YOU CAN INTERFACE WITH ESP32 WITH THIS CODE---
1. Led Blink 
2. HC-SR04 Ultrasonic Sensor
3. IR Sensor 
4. PIR Sensor
5. MPU6050 Gyro and Acceleration Sensor 
6. LM35 Temperature Sensor
7. HMC5883L Magnetometer Sensor
8. DHT11 Temperature and Humidity Sensor 
9. Soil Moisture Sensor 
10. BMP180 Barometer Sensor
11. MQ135 Air Quality Sensor
12. MQ2 Smoke Detection Sensor
13. MQ3 Alchol Detection Sensor
14. LM393 LDR Sensor
15. LM393 Sound Detection Sensor
16. Flame Detection Sensor
17. Water Level Sensor
18. Hall Sensor
*/

#include <Arduino.h> 
#include <Wire.h>             //Download library from Library Manager
#include <MPU6050_tockn.h>    //Download library from Library Manager
#include <HMC5883L_Simple.h>  //Download library from link: https://www.electronicwings.com/arduino/magnetometer-hmc5883l-interfacing-with-arduino-uno
#include <Adafruit_Sensor.h>  //Download library from Library Manager
#include <DHT.h>              //Download library from Library Manager
#include <DHT_U.h>            //Download library from Library Manager
#include <Adafruit_BMP085.h>  //Download library from Library Manager
#include <Adafruit_VL53L0X.h> //Download library from Library Manager

MPU6050 mpu6050(Wire);    //Connect SCL=A5 SDA=A4
HMC5883L_Simple Compass;  //Connect SCL=A5 SDA=A4
Adafruit_BMP085 bmp;      //Connect SCL=A5 SDA=A4


//-----CHANGE THE PIN NUMBER OR I/O PIN AS PER THE CONNECTION / REQUIREMENT-----
#define ledpin 2       //Led Pin for led blink
#define trigPin  5     //Trigger Pin 
#define echoPin  18     //Echo Pin
#define IRSensor 23     //IRSensor Pin 
#define PIRSensor 13     //PIR Sensor Pin
#define LM35_pin 33     //LM35 temperature sensor
#define DHTTYPE DHT11   //DHT11 Temperature humidity sensor Type
#define DHTPIN 15        //DHT11 Temperature humidity PIN
#define SoilMoisture 35 //Soil Moisture Sensor PIN
#define MQ135 34        //MQ135 Air quality sensor
#define MQ2 14          //MQ2 Smoke Sensor
#define MQ3 12          //MQ3 Alchol detection Sensor
#define LDR 25          //LM393 LDR sensor
#define SoundSensor 21   //LM393 Sound detector sensor
#define FlameSensor 36   //SeedStudio Flame sensor
#define WaterLevelSensor 17 //Water Level Sensor
#define HallSensor 19    //A3144 Hall effect sensor 

long timer = 0;
DHT_Unified dht(DHTPIN,DHTTYPE);
uint32_t delayMS;


void setup() {
  Serial.begin(9600);         //Begin the Serial communication
  pinMode(ledpin, OUTPUT);    //Set the ledpin as an Output 
  pinMode(trigPin, OUTPUT);   //Set the trigPin as an Output
  pinMode(echoPin, INPUT);    //Set the echoPin as an Input
  pinMode(IRSensor, INPUT);   //Set the IR data pin as an Input
  pinMode(PIRSensor, INPUT);  //Set the PIR data pin as Input
  pinMode(MQ135, INPUT);      //Set the MQ135 pin as Input
  pinMode(MQ2, INPUT);        //Set the MQ2 pin as Input
  pinMode(MQ3, INPUT);        //Set the MQ3 pin as Input
  pinMode(LDR, INPUT);        //Set the LDR pin as Input
  pinMode(SoundSensor, INPUT); //Set the Sound sensor pin as Input
  pinMode(FlameSensor, INPUT); // Set the flame sensor pin as Input
  pinMode(WaterLevelSensor, INPUT); //Set the Water Level Sensor pin as Input
  pinMode(HallSensor, INPUT); //Set the Hall Effect Sensor pin as Input
  
  
  Wire.begin();         //Begin the wire library 
  mpu6050.begin();      //Begin the MPU6050 library
  mpu6050.calcGyroOffsets(true);

  Compass.SetDeclination(-0, 23, 'W'); 
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
  

  dht.begin(); //Begin the DHT temp/humidity sensor library
  sensor_t sensor;
  delayMS = sensor.min_delay / 1000;

  bmp.begin(); //Begin the BMP Barometer sensor Library 

}

//----------PLEASE UNCOMMENT THE SENSOR WHICH YOU ARE USING WITH ARDUINO----------
void loop() {
  ledBlink();              //Uncomment to use Led blink code
  Ultrasonic();            //Uncomment to use HC-SR04 Ultrasonic sensor code
  IRsensor();              //Uncomment to use IR sensor code
  PIRsensor();             //Uncomment to use PIR sensor code
  Mpu6050();               //Uncomment to use MPU6050 sensor code 
  LM35Temprature();        //Uncomment to use LM35 Temperature code 
  HMC5883LMagnoetmeter(); //Uncomment to use HMC5883L Magnoetmeter sensor code
  DHTSensor();             //Uncomment to use DHT11 sensor code
  soilMoisture();          //Uncomment to use Soil moisture sensor code
  BMP180Barometer();       //Uncomment to use BMP180 Barometer sensor code
  MQ135AirQualitySensor(); //Uncomment to use MQ135 Air Quality sensor code
  MQ2SmokeSensor();        //Uncomment to use MQ2 Smoke Detection sensor code
  MQ3AlcholDetectionSensor();//Uncomment to use MQ3Alchol Detection sensorcode
  LM383LDRSensor();        //Uncomment to use LM393 LDR sensor code
  SoundDetector();         //Uncomment to use LM393Sound detection sensor code
  flameSensor();           //Uncomment to use SeedStudio Flame sensor code
  WaterLevel();            //Uncomment to use Water level sensor code
  HallEffectSensor();      //Uncomment to use Hall effect sensor code

}

//-----FUNCTION FOR LEDBLINK-----
void ledBlink(){
  digitalWrite(ledpin, HIGH);
  Serial.print("Led ON");
  delay(1000);
  digitalWrite(ledpin, LOW);
  Serial.print("Led OFF");
  delay(1000);
}

//-----FUNCTION FOR HC SR-04 ULTRAASONIC SENSOR-----
void Ultrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034/2;
  Serial.print("Distance: ");
  Serial.println(distance);
}

//-----FUNCTION FOR IR SENSOR-----
void IRsensor(){
  unsigned int sensorStatus = digitalRead(IRSensor);
  if (sensorStatus == 1) {
    Serial.println("Motion Detected!"); 
  }else{
    Serial.println("Motion Ended!"); 
  }
}

//-----FUNCTION FOR PIR SENSOR-----
void PIRsensor(){
  int pirState = digitalRead(PIRSensor);
  if (pirState == HIGH) {
    Serial.print("Motion Detected");
    delay(1000);
  }else {
  Serial.println("No Motion Detected");
  }
}

//-----FUNCTION FOR MPU6050 GYRO ACC SENSOR-----
void Mpu6050() {
  mpu6050.update();

  if(millis() - timer > 1000){
    
    Serial.println("=======================================================");
    Serial.print("temp : ");Serial.println(mpu6050.getTemp());
    Serial.print("accX : ");Serial.print(mpu6050.getAccX());
    Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
    Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());
  
    Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
    Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
    Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());
  
    Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
    Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());
  
    Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
    Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
    Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
    
    Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
    Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
    Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
    Serial.println("=======================================================\n");
    timer = millis(); 
  }
}

//-----FUNCTION FOR LM35 TEMPERATURE SENSOR-----
void LM35Temprature() {
  int temp_adc_val;
  float temp_val;
  temp_adc_val = analogRead(LM35_pin);// Read Temperature 
  temp_val = (temp_adc_val * 4.88);	// Convert adc value to equivalent voltage
  temp_val = (temp_val/10);	// LM35 gives output of 10mv/°C 
  Serial.print("Temperature = ");
  Serial.print(temp_val);
  Serial.print(" Degree Celsius\n");
  delay(1000);
}

//-----FUNCTION FOR HMC5883L MAGNETOMETER SENSOR-----
void HMC5883LMagnoetmeter()
{
   float heading = Compass.GetHeadingDegrees();
   Serial.print("Heading: \t");
   Serial.println( heading );   
   delay(1000);
}

//-----FUNCTION FOR DHT11 TEMPERATURE HUMIDITY SENSOR-----
void DHTSensor(){
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  Serial.print(F("Temperature: "));
  Serial.print(event.temperature);
  Serial.print(F("C"));
  dht.humidity().getEvent(&event);
  Serial.print(F("Humidity: "));
  Serial.print(event.relative_humidity);
  Serial.println(F("%"));
  delay(delayMS);
}


//-----FUNCTION FOR SOIL MOISTURE SENSOR-----
void soilMoisture(){
  float moisture_percentage;
  int sensor_analog;
  sensor_analog = analogRead(SoilMoisture);
  moisture_percentage = (100 - ((sensor_analog/1023.00)* 100));
  Serial.print("Moisture Perecentage = ");
  Serial.print(moisture_percentage);
  Serial.print("%\n\n");
  delay(1000);
}

//-----FUNCTION FOR BMP180 BAROMETER SENSOR-----
void BMP180Barometer(){
  Serial.print("Temoerature = ");
  Serial.print(bmp.readTemperature());
  Serial.print("*C");

  Serial.print("pressure = ");
  Serial.print(bmp.readPressure());
  Serial.print("Pa");

  Serial.print("Altitude = ");
  Serial.print(bmp.readAltitude());
  Serial.print(" meters");

  Serial.print("Pressure at sealevel (calculated) = ");
  Serial.print(bmp.readSealevelPressure());
  Serial.print("Pa");

  Serial.print("Real altitude = ");
  Serial.print(bmp.readAltitude(101500));
  Serial.print(" meters");

  Serial.println();
  delay(500);
}


//-----FUNCTION FOR MQ135 AIR QUALITY SENSOR-----
void MQ135AirQualitySensor(){
  int sensordata = analogRead(MQ135);
  Serial.print("Air Quality: ");
  Serial.print(sensordata, DEC);
  Serial.print(" PPM");
  delay(100);
}

//-----FUNCTION FOR MQ2 SMKODE DETECTION SENSOR-----
void MQ2SmokeSensor(){
  float sensorvalue = analogRead(MQ2);
  Serial.print("MQ2 value : ");
  Serial.print(sensorvalue);
  if (sensorvalue > 200) {
    Serial.print(" | Smoke Detection!");
  }
  Serial.println(" ");
  delay(1000);
}

//-----FUNCTION FOR MQ3 ALCHOL DETECTION SENSOR-----
void MQ3AlcholDetectionSensor(){
  float sensorV = digitalRead(MQ3);
  Serial.print("MQ3 Value: ");
  Serial.print(sensorV);
  if (sensorV < 120) {
    Serial.println("  |  Status:Stone Cold Sober");
  } else if (sensorV >= 120 && sensorV < 400) {
    Serial.print("   | Status: Drinking but within legal limits");
  }else {
    Serial.print("   | Status : DRUNK ");
  }
  delay(1000);
}

//-----FUNCTION FOR LM393 LDR SENSOR SENSOR-----
void LM383LDRSensor(){
  unsigned int SensorValue1 = analogRead(LDR);
  Serial.print("LDR Sensor : ");
  Serial.println(SensorValue1);
  delay(1000);
}

//-----FUNCTION FOR LM393 SOUND DETECTION SENSOR-----
void SoundDetector(){
  unsigned long lastSound = 0;
  int soundData = digitalRead(SoundSensor);
  if (soundData == LOW){
    if (millis() - lastSound > 25){
      Serial.println("Clap Detected!!! ");
    }
    lastSound = millis();
  }
}

//-----FUNCTION FOR FLAME SENSOR-----
void flameSensor(){
  long flamedata = digitalRead(FlameSensor);
  if (digitalRead(FlameSensor)){
    Serial.print("Flame Detected");
  }else {
    Serial.println("Flame Not Detected");
  }
  delay(1000);
}

//-----FUNCTION FOR WATER LEVEL SENSOR-----
void WaterLevel(){
  int waterValue = analogRead(WaterLevelSensor);
  if (waterValue > 570) {
    int outputValue = map(waterValue, 570, 800, 0, 255);
    Serial.print("Water Level : ");
    Serial.println(outputValue);
  }
  delay(1000);
}

//-----FUNCTION FOR HALL SENSOR-----
void HallEffectSensor(){
  int val = digitalRead(HallSensor);
  if(val == HIGH){
    Serial.print("Hall effect is active");
  }else {
    Serial.println("Hall effect is not active");
  }
}
 

