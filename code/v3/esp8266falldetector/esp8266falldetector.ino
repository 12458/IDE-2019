#include <ESP8266WiFi.h>
#include <IFTTTESP8266.h>
extern "C" {
#include "gpio.h"
}

extern "C" {
#include "user_interface.h"
}

const byte interruptPin = 2;
volatile bool interruptCounter = false;
const char personName[] = "John";

#define EVENT  "fallen"  // Put here your Maker Event Name
#define KEY  "c6TjqagKOhEczPpPtbp5Sm"  // Put here your IFTTT key
#define WIFISSID "12458Wifi"
#define PASSWORD "Pojangy1"
IFTTT client(KEY);

void setup() {
  pinMode(interruptPin, INPUT);
  ///attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);
  Serial.begin(9600);
  Serial.println("RUNNING");
  WiFi.mode(WIFI_OFF);
  delay(5);
}
/*
  void handleInterrupt() {
  interruptCounter = true;
  }*/
void sleep()
{
  wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
  gpio_pin_wakeup_enable(GPIO_ID_PIN(interruptPin), GPIO_PIN_INTR_HILEVEL); // or LO/ANYLEVEL, no change
  wifi_fpm_open();
  wifi_fpm_do_sleep(0xFFFFFFF);
}
void loop() {
  yield();
  sleep();
  delay(100);
  yield();
  if (digitalRead(interruptPin)) {
    WiFi.mode(WIFI_STA);
    delay(5);
    WiFi.begin(WIFISSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      yield();
      delay(500);
      Serial.print(".");
    }
    Serial.println("TRIGGER");
    yield();
    client.add(personName);
    client.sendAll(EVENT);
    interruptCounter = false;
    yield();
    WiFi.disconnect();
    delay(5);
    WiFi.mode(WIFI_OFF);
  }
}
