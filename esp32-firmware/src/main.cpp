/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <Wire.h>

#include "secrets.h"

#define ADDR (uint8_t) 0x28

#define BARO1_SDA 21
#define BARO1_SCL 22

#define BARO2_SDA 33
#define BARO2_SCL 32

#define PUMP_ON_PIN     4
#define VENT_VALVE_PIN  0
#define INTER_VALVE_PIN 2
#define PUMP_VALVE_PIN  15

#define BUTTON_PIN 19

TwoWire Baro1 = TwoWire(0);
TwoWire Baro2 = TwoWire(1);

hw_timer_t *my_timer = NULL;
volatile bool read_value = false;
const uint64_t step_us = 100000;
uint64_t cur_us = 0;

const unsigned long DEBOUNCE_DELAY = 50;  // Debounce time in milliseconds

int lastButtonState = HIGH;
int buttonState;
unsigned long lastDebounceTime = 0;

enum {
  STATE_IDLE,
  STATE_OPEN,
  STATE_PUMP,
  STATE_HOLD,
  STATE_MEASURE
} state = STATE_IDLE;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Baro1, OLED_RESET);

int repeat_count;

float p1, t1;
float p2, t2;

unsigned long startTime;

// Set your Static IP address
IPAddress local_IP(192, 168, 0, 104);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiServer server(80);
WiFiClient client;
bool connected = false;

const char* hostname = "pycnometer";

void IRAM_ATTR onTimer() {
  cur_us += step_us;
  read_value = true;
}

void setup() {
  Serial.begin(115200);

  Baro1.begin(BARO1_SDA, BARO1_SCL, 400000);
  Baro2.begin(BARO2_SDA, BARO2_SCL, 400000);

  my_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(my_timer, &onTimer, true);
  timerAlarmWrite(my_timer, step_us, true);
  timerAlarmEnable(my_timer);

  pinMode(PUMP_ON_PIN, OUTPUT);
  pinMode(VENT_VALVE_PIN, OUTPUT);
  pinMode(INTER_VALVE_PIN, OUTPUT);
  pinMode(PUMP_VALVE_PIN, OUTPUT);

  pinMode(BUTTON_PIN, INPUT);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.setTextSize(2); //height=9 pixels, width=5 pixels at textsize 1
  display.setTextColor(WHITE);
  display.setRotation(0); //rotates text on OLED 1=90 degrees, 2=180 degrees

  // Configuring static IP
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  // Set the hostname before connecting
  WiFi.setHostname(hostname);

  // Set up Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Start the server
  server.begin();
}

void ReadBaro(TwoWire *baro, float *p, float *t) {
  uint8_t i, val[4] = { 0, 0, 0, 0 };

  uint8_t status;             // 2 bit
  uint16_t bridge_data;       // 14 bit
  uint16_t temperature_data;  // 11 bit

  const uint16_t output_min = 0x0666;
  const uint16_t output_max = 0x3999;
  const float pressure_min = 0.;
  const float pressure_max = 103.42;

  baro->requestFrom(ADDR, (uint8_t) 4);
  for (i = 0; i <= 3; i++) {
    delay(4);                        // sensor might be missing, do not block
    val[i] = baro->read();            // by using Baro1.available()
  }

  status = (val[0] & 0xc0) >> 6;  // first 2 bits from first byte
  bridge_data = ((val[0] & 0x3f) << 8) + val[1];
  temperature_data = ((val[2] << 8) + (val[3] & 0xe0)) >> 5;

  (*p) = 1.0 * (bridge_data - output_min) * (pressure_max - pressure_min) / (output_max - output_min) + pressure_min;
  (*t) = (temperature_data / 2047. * 200.) - 50;
}

void UpdateState() {
  unsigned long curTime = millis();

  if (state == STATE_IDLE) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Idle");
    display.display();
  } else if (state == STATE_OPEN) {
    if (curTime - startTime > 5000) {
      if (repeat_count <= 3) {
        digitalWrite(INTER_VALVE_PIN, 0);
        digitalWrite(VENT_VALVE_PIN, 0);
        digitalWrite(PUMP_ON_PIN, 1);
        startTime = curTime;
        state = STATE_PUMP;
      } else {
        digitalWrite(INTER_VALVE_PIN, 0);
        digitalWrite(VENT_VALVE_PIN, 0);
        digitalWrite(PUMP_VALVE_PIN, 0);
        state = STATE_IDLE;
      }
    } else {
      display.clearDisplay();
      display.setCursor(0, 0);
      if (repeat_count <= 3) {
        display.print("#");
        display.println(repeat_count);
      } else {
        display.println("Done");
      }
      display.println("Open");
      display.print((int) ((5000 - (curTime - startTime) + 999) / 1000));
      display.println(" s");
      display.display();
    }
  } else if (state == STATE_PUMP) {
    if (curTime - startTime > 10000) {
      digitalWrite(PUMP_VALVE_PIN, 0);
      digitalWrite(PUMP_ON_PIN, 0);
      startTime = curTime;
      state = STATE_HOLD;
    } else {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("#");
      display.println(repeat_count);
      display.println("Pump");
      display.print((int) p2);
      display.println(" kPa");
      display.display();
    }
  } else if (state == STATE_HOLD) {
    if (curTime - startTime > 40000) {
      digitalWrite(INTER_VALVE_PIN, 1);
      startTime = curTime;
      state = STATE_MEASURE;
    } else {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("#");
      display.println(repeat_count);
      display.println("Hold");
      display.print((int) ((40000 - (curTime - startTime) + 999) / 1000));
      display.println(" s");
      display.display();
    }
  } else if (state == STATE_MEASURE) {
    if (curTime - startTime > 40000) {
      digitalWrite(VENT_VALVE_PIN, 1);
      digitalWrite(PUMP_VALVE_PIN, 1);
      startTime = curTime;
      ++repeat_count;
      state = STATE_OPEN;
    } else {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("#");
      display.println(repeat_count);
      display.println("Measure");
      display.print((int) ((40000 - (curTime - startTime) + 999) / 1000));
      display.println(" s");
      display.display();
    }
  }
}

void ButtonPress() {
  unsigned long curTime = millis();

  // Open valves
  digitalWrite(PUMP_ON_PIN, 0);
  digitalWrite(VENT_VALVE_PIN, 1);
  digitalWrite(INTER_VALVE_PIN, 1);
  digitalWrite(PUMP_VALVE_PIN, 1);
  startTime = curTime;

  if (state == STATE_IDLE) {
    // Start new sequence
    repeat_count = 1;
  } else {
    // Cancel
    repeat_count = 4;
  }

  state = STATE_OPEN;
}

void loop() {
  // Check if there is a new client connecting
  if (!connected) {
    client = server.available();
    if (client) {
      Serial.println("New client connected");
      connected = true;
    }
  }

  if (connected && !client.connected()) {
    client.stop();
    Serial.println("Client disconnected");
    connected = false;
  }

  if (read_value) {
    ReadBaro(&Baro1, &p1, &t1);
    ReadBaro(&Baro2, &p2, &t2);

    String data = String(cur_us / 1000000., 1) + "," +
                  String(p1, 3) + "," +
                  String(t1, 2) + "," +
                  String(p2, 3) + "," +
                  String(t2, 2);

    // Print to Wi-Fi client
    if (connected) {
        client.println(data);
    }

    read_value = false;
  }

  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        ButtonPress();
      }
    }
  }

  UpdateState();

  lastButtonState = reading;
}
