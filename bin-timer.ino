#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include <RTCVars.h>
#include <Ticker.h>
#include <LeanTask.h>

ADC_MODE(ADC_VCC);

// Define the pins for the LEDs, button, and buzzer
const uint8_t yellowLEDPin = 5; // D1
const uint8_t greenLEDPin = 4; // D2
const uint8_t redLEDPin = 14;// D5;
const uint8_t buttonPin = 12; // D6
const uint8_t buzzerPin = 13; // D7
const uint8_t resetPin = 16; // D0

// WiFi parameters
const char* ssid = "Pretty fly for a Wifi";
const char* password = "George_12";
const char* ntpServer = "pool.ntp.org";
const uint8_t timeZoneOffset = 10; //Change this to your timezone offset in hours
const uint timeUpdateInterval = 1440; // In minutes, 1440 = 1 day

// Define the states for the LEDs
const uint8_t OFF = LOW;
const uint8_t ON = HIGH;

// Buzzer, sound duration and interval
Ticker ticker;
bool buzzerOn = false;
const uint8_t buzzTime = 1; // In seconds
const uint8_t buzzerInterval = 5; // In minutes

// Define persisted state
RTCVars state; // create the state object
int sleepTime;
int recycling;

// Other variables
uint8_t btnState = 0;
uint8_t lightsTriggered = false;
const uint8_t binDay = 2;
const uint8_t buzzAfter = 20;
const uint8_t lightsOnAt = 17;
const time_t initRecyclingDate = 1715040000;
int currentHours = 0;

// Time server
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer, timeZoneOffset * 3600, timeUpdateInterval * 60000);

void blink() {
  uint8_t state = !digitalRead(redLEDPin);
  digitalWrite(redLEDPin, state);
  delay(100);
  digitalWrite(redLEDPin, !state);
}

void buzz() {
  tone(buzzerPin, 2000, buzzTime * 1000);
}

void connectWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}

void ICACHE_RAM_ATTR handleButtonPress() {
  if (btnState != 1 && lightsTriggered) {
    btnState = 1;
  }
}

void buttonPressTasks() {
    Serial.println("Button press!");

    // Turn off all LEDs & buzzer
    digitalWrite(redLEDPin, OFF);
    digitalWrite(yellowLEDPin, OFF);
    digitalWrite(greenLEDPin, OFF);

    sleepTime = 1;
    state.saveToRTC();
    ESP.deepSleep(SECS_PER_HOUR * 1000000, WAKE_RF_DISABLED);
}

void fullBoot() {
  connectWiFi();

  // Get time from server
  timeClient.begin();
  timeClient.update();
}

uint16_t voltage() {
  uint16_t volt = ESP.getVcc();
  Serial.println("Voltage: " + (String)volt);
  return volt;
}

void powerSaving() {
  if (sleepTime > 1) {
    Serial.println("Going to sleep... " + (String)sleepTime);
    ESP.deepSleepInstant(SECS_PER_MIN * 1000000, WAKE_RF_DISABLED);
  } else if (sleepTime == 1) {
    Serial.println("Going to sleep, one last time...");
    ESP.deepSleepInstant(SECS_PER_HOUR * 1000000, WAKE_RFCAL);
  }
}

const int tempo = 124;
const int baseDuration = 60 * 1000 / tempo;
float duration[] = {1.5, 0.5, 1, 1, 0.5, 2, 1};
int frequency[] = {220, 220, 247, 220, 196, 175, 165};

void playTune() {
  for (int i = 0; i < (sizeof(duration)/sizeof(*duration)); i++) {
    float d = baseDuration * duration[i];
    tone(buzzerPin, frequency[i], d);
    delay(d * 1.2);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  pinMode(redLEDPin, OUTPUT);
  pinMode(yellowLEDPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(resetPin, INPUT);
  pinMode(buzzerPin, OUTPUT);

  digitalWrite(redLEDPin, OFF);
  digitalWrite(yellowLEDPin, OFF);
  digitalWrite(greenLEDPin, OFF);

  // Create button interrupt
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, RISING);

  // Load state from RTC
  state.registerVar(&sleepTime);
  state.registerVar(&recycling);

  if (state.loadFromRTC()) {
    sleepTime--;
    state.saveToRTC();

    // powerSaving();
    if (voltage() < 3100) {
      // Low voltage recharge battery
      playTune();
      ticker.attach_scheduled(5, blink);
    }

    fullBoot();
  } else {
    Serial.println("Cold boot, no RTC memory.");

    fullBoot();
    // Calculate if recycling or green waste
    const long diff = abs(initRecyclingDate - timeClient.getEpochTime());
    const int recyclingDays = diff / SECS_PER_DAY;
    recycling = (recyclingDays % 14) < 7;
    sleepTime = 0;
    state.saveToRTC();
  }

  // Calculate number of times to sleep before next recycling date after time set
  // Make sure if woken up out of cycle to reconnect and determine cycles again
  // Or if time too far away once cycles complete to reinitialise more sleep cycles and enter sleep cycle again
}

void loop() {
  timeClient.update();

  if (btnState == 1) buttonPressTasks();

  int dayOfWeek = timeClient.getDay(); // Get the current day of the week (from 0-6 where 0=Sunday)
  currentHours = timeClient.getHours();

  // Check for bin night and turn on the appropriate LED
  if (dayOfWeek == binDay && !lightsTriggered) {
    digitalWrite(redLEDPin, ON); // Rubbish every Tuesday
    Serial.println(recycling);
    if (recycling == 1) {
      digitalWrite(yellowLEDPin, ON); // Recycling light every fortnight
      recycling = 0;
    } else {
      digitalWrite(greenLEDPin, ON);  // Green waste every other fornight
      recycling = 1;
    }
    state.saveToRTC();
    lightsTriggered = true;
  }

  if (lightsTriggered == 1 && !buzzerOn) {
    ticker.attach_scheduled(buzzerInterval * 60, buzz);
    buzzerOn = true;
  }
}

