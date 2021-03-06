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
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

//int data[STORE_SIZE][5]; //array for saving past data
//byte currentIndex=0; //stores current data array index (0-255)
bool fall = false; //stores if a fall has occurred
bool trigger1 = false; //stores if first trigger (lower threshold) has occurred
bool trigger2 = false; //stores if second trigger (upper threshold) has occurred
bool trigger3 = false; //stores if third trigger (orientation change) has occurred

int angleChange = 0;
byte buttonState;
byte lastbuttonState;
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
}
void loop() {
  emergencyButton();
  mpu_read();
  //Calibration
  ax = (AcX + 800) / 16384.00;
  ay = (AcY - 2718) / 16384.00;
  az = (AcZ + 1145) / 16384.00;

  //Gyro Calibration
  gx = (GyX + 50) / 131.07;
  gy = (GyY + 44) / 131.07;
  gz = (GyZ + 32) / 131.07;

  // calculating Amplitute vactor for 3 axis
  float Raw_AM = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
  int AM = Raw_AM * 10;  // as values are within 0 to 1, I multiplied
  Serial.print("AM:");
  Serial.println(AM);
  Serial.print("Angle Change:");
  Serial.println(angleChange);
  // it by for using if else conditions
  if (AM <= 10 && trigger2 == false) { //if AM breaks lower threshold (0.4g)
    trigger1 = true;
    Serial.println("\r TRIGGER 1 ACTIVATED");
  }
  //  Serial.println(AM);
  if (trigger1 == true) {
    if (AM >= 25) { //if AM breaks upper threshold (3g)
      trigger2 = true;
      trigger1 = false;
    }
  }

  if (trigger2 == true) {
    //angleChange=acos(((double)x*(double)bx+(double)y*(double)by+(double)z*(double)bz)/(double)AM/(double)BM);
    angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
    if (angleChange >= 30) { //if orientation changes by between 80-100 degrees
      fall = true;
      trigger2 = false;
      Serial.println("ANGLE Change:");
      Serial.print(angleChange);
    }
  }
  if (fall == true) { //in event of a fall detection
    Serial.println("Fall Triggered");
    digitalWrite(fallTrigger, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(20);
    digitalWrite(fallTrigger, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    fall = false;
  }
  //It appears that delay is needed in order not to clog the port
  delay(10);
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

void emergencyButton() {
  buttonState = digitalRead(button);
  if (buttonState == LOW && buttonState != lastbuttonState) {
    // message:
    Serial.println("The emergency button has been pressed.");
    fall = true;
  }
  lastbuttonState = buttonState;
}
