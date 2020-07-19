#include <Wire.h>
#include <MPU6050.h>
#include <I2Cdev.h>

int buttonState;
int lastbuttonState;
const byte button = 5;
const byte fallTrigger = 11;
int pin = 13;
volatile int state = LOW;
bool fall = false;
MPU6050 accelgyro;         // Declare the instanced of the MPU6050

void setup()
{
  Serial.begin(9600);
  pinMode(button, INPUT);
  pinMode(fallTrigger, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(fallTrigger, LOW);
  Wire.begin();
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  pinMode(pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), fallDetection, RISING);
  // Setup interrupt Config
  accelgyro.setInterruptMode(0); // active high
  accelgyro.setInterruptDrive(0); // push pull
  accelgyro.setInterruptLatch(1); // latch until read
  accelgyro.setInterruptLatchClear(1); // clear on any read
  accelgyro.setIntDataReadyEnabled(false); // trigger interrupt on data ready
  accelgyro.setXAccelOffset(800);
  accelgyro.setYAccelOffset(-2718);
  accelgyro.setZAccelOffset(1145);
  accelgyro.setXGyroOffset(50);
  accelgyro.setXGyroOffset(44);
  accelgyro.setXGyroOffset(32);

  //Fall interrupt setup
  accelgyro.setIntFreefallEnabled(1);
  accelgyro.setFreefallDetectionThreshold(0x10);
  accelgyro.setFreefallDetectionDuration(0x05);

}

void loop()
{
  buttonState = digitalRead(button);
  if (buttonState == LOW && buttonState != lastbuttonState) {
    // message:
    Serial.println("Emergency Button Pressed");
    fall = true;
  }
  lastbuttonState = buttonState;
  if (fall) {
    Serial.println("Fall Triggered");
    digitalWrite(fallTrigger, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(20);
    digitalWrite(fallTrigger, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    fall = false;
  }
}

void fallDetection()
{
  fall = true;
}
