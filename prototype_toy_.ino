#include <MAX3010x.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include "filters.h"
#include <DHT.h>
#include <BluetoothSerial.h>

// OLED SPI Pins
#define OLED_MOSI   23
#define OLED_CLK    18
#define OLED_DC     16
#define OLED_CS     5
#define OLED_RESET  17

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS);

// DHT11
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Bluetooth
BluetoothSerial SerialBT;
String btBuffer = "";

// Time & Date Variables
int hourNow = 0, minuteNow = 0, dayNow = 0, monthNow = 0, yearNow = 0;
unsigned long lastMinuteUpdate = 0;
bool timeSet = false;

// Sensor
MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;
const float kEdgeThreshold = -2000.0;

const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;
const bool kEnableAveraging = true;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;

LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

long last_heartbeat = 0;
long finger_timestamp = 0;
bool finger_detected = false;
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

void setup() {
  Serial.begin(9600);
  SerialBT.begin("Genius_MRIDUL_G");
  if (!display.begin(SSD1306_SWITCHCAPVCC)) while (1);
  if (!sensor.begin() || !sensor.setSamplingRate(kSamplingRate)) while (1);
  dht.begin();
  display.clearDisplay();
  display.display();
}

void loop() {
  receiveBluetoothData();
  updateTimeFromMillis();

  float tempC = dht.readTemperature();
  auto sample = sensor.readSample(1000);
  float red = sample.red;
  float ir = sample.ir;

  if (red > kFingerThreshold) {
    if (millis() - finger_timestamp > kFingerCooldownMs) finger_detected = true;
  } else {
    resetFilters();
    finger_detected = false;
    finger_timestamp = millis();
  }

  if (finger_detected) {
    red = low_pass_filter_red.process(red);
    ir = low_pass_filter_ir.process(ir);
    stat_red.process(red);
    stat_ir.process(ir);
    float filtered = high_pass_filter.process(red);
    float diff = differentiator.process(filtered);

    if (!isnan(diff) && !isnan(last_diff)) {
      if (last_diff > 0 && diff < 0) {
        crossed = true;
        crossed_time = millis();
      }
      if (diff > 0) crossed = false;

      if (crossed && diff < kEdgeThreshold) {
        if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
          int bpm = 60000 / (crossed_time - last_heartbeat);
          float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
          float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();
          float r = rred / rir;
          float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
          if (bpm > 50 && bpm < 250) {
            if (kEnableAveraging) {
              int avgBpm = averager_bpm.process(bpm);
              int avgSpO2 = averager_spo2.process(spo2);
              if (averager_bpm.count() >= kSampleThreshold) {
                displayMeasuredValues(false, avgBpm, avgSpO2, tempC);
              }
            } else {
              displayMeasuredValues(false, bpm, spo2, tempC);
            }
          }
          stat_red.reset();
          stat_ir.reset();
        }
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }
    last_diff = diff;
  } else {
    displayMeasuredValues(true, 0, 0, tempC);
  }
}

void resetFilters() {
  differentiator.reset();
  averager_bpm.reset();
  averager_r.reset();
  averager_spo2.reset();
  low_pass_filter_red.reset();
  low_pass_filter_ir.reset();
  high_pass_filter.reset();
  stat_red.reset();
  stat_ir.reset();
}

void receiveBluetoothData() {
  while (SerialBT.available()) {
    char ch = SerialBT.read();
    if (ch == '\n') {
      if (!timeSet && btBuffer.startsWith("T: ")) {
        String payload = btBuffer.substring(3);
        int d, m, y, h, min;
        if (sscanf(payload.c_str(), "%d/%d/%d %d:%d", &d, &m, &y, &h, &min) == 5) {
          dayNow = d; monthNow = m; yearNow = y;
          hourNow = h; minuteNow = min;
          lastMinuteUpdate = millis();
          timeSet = true;
        }
      }
      btBuffer = "";
    } else {
      btBuffer += ch;
    }
  }
}

void updateTimeFromMillis() {
  if (!timeSet) return;
  unsigned long now = millis();
  if (now - lastMinuteUpdate >= 60000) {
    minuteNow++;
    if (minuteNow >= 60) {
      minuteNow = 0;
      hourNow++;
      if (hourNow >= 24) {
        hourNow = 0;
        dayNow++;  // You can add month/day logic here if needed
      }
    }
    lastMinuteUpdate += 60000;
  }
}

void displayMeasuredValues(bool noFinger, int bpm, int spo2, float tempC) {
  display.setRotation(2);//---->This line...
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  int16_t x1, y1;
  uint16_t w, h;

  String timeStr = (hourNow < 10 ? "0" : "") + String(hourNow) + ":" + (minuteNow < 10 ? "0" : "") + String(minuteNow);
  String dateStr = (dayNow < 10 ? "0" : "") + String(dayNow) + "/" + (monthNow < 10 ? "0" : "") + String(monthNow) + "/" + String(yearNow);

  display.getTextBounds(timeStr, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 0);
  display.print(timeStr);

  display.getTextBounds(dateStr, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 10);
  display.print(dateStr);

  String tempStr = "Temp: ";
  if (!isnan(tempC)) tempStr += String(tempC, 1) + " C";
  else tempStr += "-- C";
  display.getTextBounds(tempStr, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 22);
  display.print(tempStr);

  String statusStr;
  if (noFinger) statusStr = "Initializing sir!";
  else if (bpm < 30) statusStr = "Pls. Wait";
  else {
    statusStr = "BPM: " + String(bpm) + " SpO2: ";
    statusStr += (spo2 >= 20 && spo2 <= 100) ? String(spo2) : "--";
  }
  display.getTextBounds(statusStr, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 38);
  display.print(statusStr);

  String footer = "Master Mridul!";
  display.getTextBounds(footer, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, SCREEN_HEIGHT - h);
  display.print(footer);

  display.display();
}
