//IDE 2019 Falll detector
/*
   Pins Used
   5 -emergency button
   11 -esp8266 fall trigger
   I2C Bus
   - A4 SDA
   - A5 SCL
*/
#include <Wire.h>
//#include "MAX30105.h"
//#include "heartRate.h"
//#include "spo2_algorithm.h"

const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

//int data[STORE_SIZE][5]; //array for saving past data
//byte currentIndex=0; //stores current data array index (0-255)
boolean fall = false; //stores if a fall has occurred
boolean trigger1 = false; //stores if first trigger (lower threshold) has occurred
boolean trigger2 = false; //stores if second trigger (upper threshold) has occurred
boolean trigger3 = false; //stores if third trigger (orientation change) has occurred

byte trigger1count = 0; //stores the counts past since trigger 1 was set true
byte trigger2count = 0; //stores the counts past since trigger 2 was set true
byte trigger3count = 0; //stores the counts past since trigger 3 was set true
int angleChange = 0;
int buttonState;
int lastbuttonState;
const byte button = 5;
const byte fallTrigger = 11;

//MAX30105 particleSensor;
///unsigned long previousMillis = 0;
//unsigned long currentMillis;
//const unsigned long period = 1000;  //the value is a number of milliseconds
//float beatsPerMinute = 0;
//const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
//byte rates[RATE_SIZE]; //Array of heart rates
///byte rateSpot = 0;
//long lastBeat = 0; //Time at which the last beat occurred
//byte beatAvg = 0;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

  pinMode(button, INPUT);
  pinMode(fallTrigger, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(fallTrigger, LOW);
  //digitalWrite(11, HIGH);
  /*
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
    {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
    }
    Serial.println("Place your index finger on the sensor with steady pressure.");*/

  //particleSensor.setup(); //Configure sensor with default settings
  //particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  //particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}
void loop() {

  //  heartime();
  whatname();
  mpu_read();
  //2050, 77, 1947 are values for calibration of accelerometer
  // values may be different for you
  ax = (AcX + 800) / 16384.00;
  ay = (AcY - 2718) / 16384.00;
  az = (AcZ + 1145) / 16384.00;

  //270, 351, 136 for gyroscope
  gx = (GyX + 50) / 131.07;
  gy = (GyY + 44) / 131.07;
  gz = (GyZ + 32) / 131.07;

  // calculating Amplitute vactor for 3 axis
  float Raw_AM = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  int AM = Raw_AM * 10;  // as values are within 0 to 1, I multiplied
  // it by for using if else conditions

  Serial.println(AM);
  //Serial.println(PM);
  //delay(500);

  if (trigger3 == true) {
    trigger3count++;
    //Serial.println(trigger3count);
    if (trigger3count >= 10) {
      angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
      //delay(10);
      Serial.println(angleChange);
      if ((angleChange >= 0) && (angleChange <= 10)) { //if orientation changes remains between 0-10 degrees
        fall = true;
        trigger3 = false;
        trigger3count = 0;
        Serial.println("ANGLE Change:");
        Serial.print(angleChange);
      }
      else { //user regained normal orientation
        trigger3 = false; 
        trigger3count = 0;
        Serial.println("TRIGGER 3 DEACTIVATED");
      }
    }
  }
  if (fall == true) { //in event of a fall detection
    Serial.println("FALL DETECTED");
    digitalWrite(fallTrigger, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(20);
    digitalWrite(fallTrigger, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    fall = false;
    // exit(1);
  }
  if (trigger2count >= 6) { //allow 0.5s for orientation change
    trigger2 = false;
    trigger2count = 0;
    Serial.println("TRIGGER 2 DECACTIVATED");
  }
  if (trigger1count >= 6) { //allow 0.5s for AM to break upper threshold
    trigger1 = false;
    trigger1count = 0;
    Serial.println("TRIGGER 1 DECACTIVATED");
  }
  if (trigger2 == true) {
    trigger2count++;
    //angleChange=acos(((double)x*(double)bx+(double)y*(double)by+(double)z*(double)bz)/(double)AM/(double)BM);
    angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5); Serial.println(angleChange);
    if (angleChange >= 30 && angleChange <= 400) { //if orientation changes by between 80-100 degrees
      trigger3 = true;
      trigger2 = false;
      trigger2count = 0;
      Serial.println("ANGLE Change:");
      Serial.print(angleChange);
      Serial.println("TRIGGER 3 ACTIVATED");
    }
  }
  if (trigger1 == true) {
    trigger1count++;
    if (AM >= 12) { //if AM breaks upper threshold (3g)
      trigger2 = true;
      Serial.println("TRIGGER 2 ACTIVATED");
      trigger1 = false; trigger1count = 0;
    }
  }
  if (AM <= 2 && trigger2 == false) { //if AM breaks lower threshold (0.4g)
    trigger1 = true;
    Serial.println("TRIGGER 1 ACTIVATED");
  }
  //It appears that delay is needed in order not to clog the port
  delay(100);
}

void mpu_read() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void whatname() {
  buttonState = digitalRead(button);
  if (buttonState == LOW && buttonState != lastbuttonState) {
    // message:
    Serial.println("The emergency button has been pressed.");
    fall = true;
  }
  lastbuttonState = buttonState;
}
/*
  void heartime()
  {
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue) == true) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  }*/
