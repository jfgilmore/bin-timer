#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <RTCVars.h>
#include <Scheduler.h>
#include <LeanTask.h>

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

// Buzzer Sound duration and interval
const uint8_t buzzTime = 1; // In seconds
const uint8_t buzzInterval = 5; // In minutes

// Define persisted state
RTCVars state; // create the state object
int sleepTime;
int recycling;

// Other variables
uint8_t btnState = 0;
uint8_t lightsTriggered = 0;
const uint8_t binDay = 5;
const uint8_t buzzAfter = 20;
const uint8_t lightsOnAt = 17;
const time_t initRecyclingDate = 1715040000;
int currentHours = 0;

// Time server
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer, timeZoneOffset * 3600, timeUpdateInterval * 60000);

class BuzzerTask : public LeanTask {
  protected:
    void setup() {
      pinMode(buzzerPin, OUTPUT);
      noTone(buzzerPin);
    }

    void loop() {
      if (!(lightsTriggered == 1 && currentHours >= buzzAfter)) {
        noTone(buzzerPin);
      } else {
        // If it's past 8 PM, start buzzing every 10 minutes until the button is pressed
        beepBuzzer();
        delay(buzzInterval * 60 * 1000);
      }
    }

    void beepBuzzer() {
      tone(buzzerPin, 2000, buzzTime * 1000);
    }
} buzzer_task;


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
  if (btnState != 1 && lightsTriggered == 1) {
    btnState = 1;
  }
}

void buttonPressTasks() {
    Serial.println("Button press!");

    // Turn off all LEDs & buzzer
    digitalWrite(redLEDPin, OFF);
    digitalWrite(yellowLEDPin, OFF);
    digitalWrite(greenLEDPin, OFF);

    btnState = 0;
    sleepTime = 1;
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  pinMode(redLEDPin, OUTPUT);
  pinMode(yellowLEDPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(resetPin, INPUT);

  digitalWrite(redLEDPin, OFF);
  digitalWrite(yellowLEDPin, OFF);
  digitalWrite(greenLEDPin, OFF);

  // Create button interrupt
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, RISING);

  // Load state from RTC
  state.registerVar(&sleepTime);
  state.registerVar(&recycling);

  if (!state.loadFromRTC()) {
    // if (sleepTime > 0) {
    //   sleepTime++;
    //   state.saveToRTC();
    // }
    // Serial.println("This is reset no. " + (String)(sleepTime));
  // } else {
    Serial.println("Cold boot, no RTC memory.");
    connectWiFi();
    // Get time from server
    timeClient.begin();
    timeClient.update();

    // Calculate if recycling or green waste
    const long diff = abs(initRecyclingDate - timeClient.getEpochTime());
    const int recyclingDays = diff / SECS_PER_DAY;
    recycling = (recyclingDays % 14) < 7;
    sleepTime = 0;
    state.saveToRTC();
  }

  Scheduler.start(&buzzer_task);
  Scheduler.begin();
}

void loop() {
  timeClient.update();
  yield();
  if (btnState == 1) buttonPressTasks();

  int dayOfWeek = timeClient.getDay(); // Get the current day of the week (from 0-6 where 0=Sunday)
  currentHours = timeClient.getHours();

  if (dayOfWeek == binDay && currentHours > lightsOnAt) {
    sleepTime = 0;
  }

  if (sleepTime == 1) {
    // const WakeMode mode = currentHours < lightsOnAt - 1 ? WAKE_RF_DISABLED : RF_CAL;
    Serial.println("Time is: " + timeClient.getFormattedTime());
    Serial.println("Going to sleep...");
    state.saveToRTC();
    ESP.deepSleep(SECS_PER_MIN * 1000000);
  }

  if (!WiFi.isConnected()) {
    connectWiFi();
  }

  // Check for bin night and turn on the appropriate LED
  if (dayOfWeek == binDay && lightsTriggered == 0) {
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
    lightsTriggered = 1;
  }
}

