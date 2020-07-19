#include <ESP8266WiFi.h>
#include <IFTTTESP8266.h>
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
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);
  Serial.begin(9600);
  Serial.println("RUNNING");
  WiFi.setAutoReconnect(1);
  WiFi.mode(WIFI_OFF);
  WiFi.setSleepMode(WIFI_MODEM_SLEEP);
  WiFi.forceSleepBegin();
  delay(1);
}
void handleInterrupt() {
  interruptCounter = true;
}
void loop() {
  if (interruptCounter) {
    WiFi.forceSleepWake();
    delay(1);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFISSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("TRIGGER");
    client.add(personName);
    client.sendAll(EVENT);
    interruptCounter = false;
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();
    delay(1);
  }
}
