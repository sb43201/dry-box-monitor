#include <Arduino.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Preferences.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <XPT2046_Touchscreen.h>
#include <ESPmDNS.h>
#include <esp_now.h>

#include "protocol.h"
#include "weather.h"

namespace {
constexpr uint8_t TFT_BACKLIGHT = 27;
constexpr uint8_t TOUCH_CS_PIN = 33;
constexpr uint8_t TOUCH_IRQ = 36;
constexpr uint8_t TOUCH_SCK = 14;
constexpr uint8_t TOUCH_MISO = 12;
constexpr uint8_t TOUCH_MOSI = 13;
constexpr uint8_t SCREEN_ROTATION = 0;
constexpr int16_t SCREEN_WIDTH = 320;
constexpr int16_t SCREEN_HEIGHT = 480;
constexpr int DEFAULT_TOUCH_MIN_X = 300;
constexpr int DEFAULT_TOUCH_MAX_X = 3800;
constexpr int DEFAULT_TOUCH_MIN_Y = 280;
constexpr int DEFAULT_TOUCH_MAX_Y = 3850;
constexpr uint32_t OFFLINE_AFTER_MS = 120000;
constexpr uint32_t OVERVIEW_REFRESH_MS = 30000;
constexpr uint32_t DETAIL_REFRESH_MS = 1000;
constexpr uint32_t PACKET_REDRAW_DELAY_MS = 250;
constexpr uint8_t LOCAL_I2C_SDA = 32;
constexpr uint8_t LOCAL_I2C_SCL = 25;
constexpr uint32_t LOCAL_SENSOR_INTERVAL_MS = 30000;
constexpr uint32_t WIFI_CHOICE_TIMEOUT_MS = 8000;
constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 10000;
constexpr uint8_t WIFI_RECONNECTS_BEFORE_RESTART = 6;
constexpr uint32_t HISTORY_BUCKET_MS = 5UL * 60UL * 1000UL;
constexpr uint16_t HISTORY_BUCKETS = 24 * 60 / 5;
constexpr uint8_t BROADCAST_ADDRESS[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

constexpr uint16_t COLOR_BG = 0x0841;
constexpr uint16_t COLOR_PANEL = 0x10A2;
constexpr uint16_t COLOR_HEADER = 0x018C;
constexpr uint16_t COLOR_MUTED = 0xAD55;
constexpr uint16_t COLOR_GOOD = 0x05E0;
constexpr uint16_t COLOR_WARN = 0xFD20;
constexpr uint16_t COLOR_BAD = 0xF800;

struct NodeState {
  DryBoxProtocol::SensorPacket packet{};
  uint32_t receivedMs = 0;
  bool received = false;
};

struct PendingPairRequest {
  DryBoxProtocol::PairRequest request{};
  uint8_t sourceMac[6]{};
  uint8_t receivedChannel = 1;
  bool pending = false;
};

struct HistorySample {
  uint32_t bucket = 0;
  int16_t temperatureCenti = 0;
  uint16_t humidityCenti = 0;
};

struct NodeHistory {
  HistorySample samples[HISTORY_BUCKETS]{};
  uint16_t head = 0;
  uint16_t count = 0;
  uint32_t lastBucket = UINT32_MAX;
};

struct PendingReadingAck {
  DryBoxProtocol::ReadingAck ack{};
  uint8_t destinationMac[6]{};
  uint32_t readyAtMs = 0;
  bool pending = false;
};

struct LocalSensorReading {
  float rawTemperatureC = NAN;
  float temperatureC = NAN;
  float humidityRh = NAN;
  float bmpTemperatureC = NAN;
  float pressureHpa = NAN;
  bool ahtOk = false;
  bool bmpOk = false;
};

TFT_eSPI tft;
SPIClass touchSpi(HSPI);
XPT2046_Touchscreen touch(TOUCH_CS_PIN, TOUCH_IRQ);
Preferences preferences;
NodeState nodes[DryBoxProtocol::NODE_COUNT];
NodeHistory nodeHistory[DryBoxProtocol::NODE_COUNT];
uint8_t pairedMacs[DryBoxProtocol::NODE_COUNT][6]{};
bool slotPaired[DryBoxProtocol::NODE_COUNT]{};
uint8_t localMac[6]{};
PendingPairRequest pendingPair;
PendingReadingAck pendingReadingAcks[DryBoxProtocol::NODE_COUNT];
uint8_t ackQueueHead = 0;
uint8_t ackQueueTail = 0;
uint8_t ackQueueCount = 0;
uint32_t lastAckQueuedMs[DryBoxProtocol::NODE_COUNT]{};
portMUX_TYPE nodeMux = portMUX_INITIALIZER_UNLOCKED;
float goodLimitRh = 30.0f;
float warningLimitRh = 45.0f;
uint32_t validPacketCount = 0;
uint32_t lastRedrawMs = 0;
uint32_t lastTouchMs = 0;
int8_t detailNode = -1;
bool settingsScreen = false;
bool pairingScreen = false;
bool pairingActive = false;
uint8_t selectedPairSlot = 0;
uint32_t pairingEndsMs = 0;
volatile bool packetPendingRedraw = false;
uint8_t overviewPage = 0;
String hostname = "drybox-monitor";
WebServer webServer(80);
bool webServerStarted = false;
bool weatherScreen = false;
String weatherApiKey;
String weatherLatitude = "39.7684";
String weatherLongitude = "-86.1581";
String weatherLocation = "Indianapolis";
String weatherTimezone = "EST5EDT,M3.2.0,M11.1.0";
bool weatherImperial = true;
WeatherClient weatherClient;
WeatherData weatherData;
String weatherError;
uint32_t lastWeatherAttemptMs = 0;
uint32_t lastPairBeaconMs = 0;
uint32_t wifiDisconnectedSinceMs = 0;
uint32_t lastWifiReconnectAttemptMs = 0;
uint8_t wifiReconnectAttempts = 0;
uint8_t lastConnectedWifiChannel = 0;
bool wifiWasConnected = false;
bool wifiSkippedAtStartup = false;
constexpr uint32_t WEATHER_INTERVAL_MS = 20UL * 60UL * 1000UL;
Adafruit_AHTX0 localAht;
Adafruit_BMP280 localBmp;
LocalSensorReading localReading;
uint32_t lastLocalSensorMs = 0;
bool localAhtFound = false;
bool localBmpFound = false;
float localTemperatureOffsetC = 0.0f;
int touchMinX = DEFAULT_TOUCH_MIN_X;
int touchMaxX = DEFAULT_TOUCH_MAX_X;
int touchMinY = DEFAULT_TOUCH_MIN_Y;
int touchMaxY = DEFAULT_TOUCH_MAX_Y;
bool touchCalibrated = false;

void drawHeader(const char *title, const char *right = nullptr);

void drawCalibrationTarget(uint8_t step, bool complete = false) {
  tft.fillScreen(COLOR_BG);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(COLOR_GOOD, COLOR_BG);
  tft.drawString("TOUCH CALIBRATION", SCREEN_WIDTH / 2, 18, 4);
  tft.setTextColor(COLOR_MUTED, COLOR_BG);
  tft.drawString(complete ? "Calibration saved" : "Tap and release each target", SCREEN_WIDTH / 2, 58, 2);
  if (complete) {
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.drawString("Returning to Settings...", SCREEN_WIDTH / 2, 210, 4);
    tft.setTextDatum(TL_DATUM);
    return;
  }
  constexpr int16_t margin = 28;
  const int16_t xs[] = {margin, SCREEN_WIDTH - margin, SCREEN_WIDTH - margin, margin};
  const int16_t ys[] = {60, 60, SCREEN_HEIGHT - margin, SCREEN_HEIGHT - margin};
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.drawString("Point " + String(step + 1) + " of 4", SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, 2);
  tft.drawCircle(xs[step], ys[step], 16, COLOR_WARN);
  tft.drawFastHLine(xs[step] - 22, ys[step], 45, COLOR_WARN);
  tft.drawFastVLine(xs[step], ys[step] - 22, 45, COLOR_WARN);
  tft.setTextDatum(TL_DATUM);
}

void runTouchCalibration() {
  TS_Point points[4];
  for (uint8_t step = 0; step < 4; ++step) {
    drawCalibrationTarget(step);
    while (touch.touched()) delay(10);
    while (!touch.touched()) delay(10);
    points[step] = touch.getPoint();
    while (touch.touched()) delay(10);
    delay(80);
  }
  const int leftX = (points[0].x + points[3].x) / 2;
  const int rightX = (points[1].x + points[2].x) / 2;
  const int topY = (points[0].y + points[1].y) / 2;
  const int bottomY = (points[2].y + points[3].y) / 2;
  if (abs(rightX - leftX) >= 300 && abs(bottomY - topY) >= 300) {
    constexpr float targetLeft = 28.0f;
    constexpr float targetRight = SCREEN_WIDTH - 28.0f;
    constexpr float targetTop = 60.0f;
    constexpr float targetBottom = SCREEN_HEIGHT - 28.0f;
    const float xScale = (rightX - leftX) / (targetRight - targetLeft);
    const float yScale = (bottomY - topY) / (targetBottom - targetTop);
    touchMinX = lroundf(leftX - xScale * targetLeft);
    touchMaxX = lroundf(leftX + xScale * ((SCREEN_WIDTH - 1) - targetLeft));
    touchMinY = lroundf(topY - yScale * targetTop);
    touchMaxY = lroundf(topY + yScale * ((SCREEN_HEIGHT - 1) - targetTop));
    touchCalibrated = true;
    preferences.begin("drybox", false);
    preferences.putInt("touchMinX", touchMinX);
    preferences.putInt("touchMaxX", touchMaxX);
    preferences.putInt("touchMinY", touchMinY);
    preferences.putInt("touchMaxY", touchMaxY);
    preferences.putBool("touchCal", true);
    preferences.end();
    drawCalibrationTarget(0, true);
    delay(900);
  } else {
    runTouchCalibration();
  }
  lastTouchMs = millis();
}

void readLocalSensor() {
  lastLocalSensorMs = millis();
  if (!localAhtFound) localAhtFound = localAht.begin(&Wire);
  if (!localBmpFound) localBmpFound = localBmp.begin(0x76, 0x58) || localBmp.begin(0x77, 0x58);
  localReading.ahtOk = false;
  localReading.bmpOk = false;
  if (localAhtFound) {
    sensors_event_t humidity;
    sensors_event_t temperature;
    localAht.getEvent(&humidity, &temperature);
    localReading.ahtOk = isfinite(temperature.temperature) && isfinite(humidity.relative_humidity);
    if (localReading.ahtOk) {
      localReading.rawTemperatureC = temperature.temperature;
      localReading.temperatureC = localReading.rawTemperatureC + localTemperatureOffsetC;
      localReading.humidityRh = humidity.relative_humidity;
    }
  }
  if (localBmpFound) {
    const float bmpTemperature = localBmp.readTemperature();
    const float pressure = localBmp.readPressure() / 100.0f;
    localReading.bmpOk = isfinite(bmpTemperature) && isfinite(pressure) && pressure > 300.0f && pressure < 1200.0f;
    if (localReading.bmpOk) {
      localReading.bmpTemperatureC = bmpTemperature;
      localReading.pressureHpa = pressure;
    }
  }
  if (localReading.ahtOk) {
    const float rawF = localReading.rawTemperatureC * 9.0f / 5.0f + 32.0f;
    const float correctedF = localReading.temperatureC * 9.0f / 5.0f + 32.0f;
    Serial.printf("[AHT20] raw=%.2fC/%.2fF corrected=%.2fC/%.2fF offset=%+.2fC humidity=%.2f%%\n",
                  localReading.rawTemperatureC, rawF, localReading.temperatureC, correctedF,
                  localTemperatureOffsetC, localReading.humidityRh);
  } else {
    Serial.println("[AHT20] temperature/humidity read failed");
  }
  if (localReading.bmpOk) {
    const float bmpTemperatureF = localReading.bmpTemperatureC * 9.0f / 5.0f + 32.0f;
    Serial.printf("[BMP280] raw=%.2fC/%.2fF pressure=%.2fhPa\n", localReading.bmpTemperatureC, bmpTemperatureF,
                  localReading.pressureHpa);
  } else {
    Serial.println("[BMP280] temperature/pressure read failed");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[wifi] ssid=%s rssi=%ddBm channel=%d ip=%s hostname=%s\n", WiFi.SSID().c_str(), WiFi.RSSI(),
                  WiFi.channel(), WiFi.localIP().toString().c_str(), hostname.c_str());
  } else {
    Serial.printf("[wifi] disconnected status=%d\n", static_cast<int>(WiFi.status()));
  }
}

const char *nodeName(uint8_t index) {
  static const char *names[] = {"ACE 1", "ACE 2", "ACE 3", "ACE 4", "ACE 5",
                                "ACE 6", "ACE 7", "ACE 8", "ACE 9", "ACE 10"};
  return names[index];
}

String sanitizeHostname(String value) {
  value.trim();
  value.toLowerCase();
  if (value.endsWith(".local")) value.remove(value.length() - 6);
  String clean;
  for (size_t i = 0; i < value.length() && clean.length() < 32; ++i) {
    const char c = value[i];
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
        (c == '-' && clean.length() && !clean.endsWith("-"))) clean += c;
  }
  while (clean.endsWith("-")) clean.remove(clean.length() - 1);
  return clean.length() ? clean : String("drybox-monitor");
}

void drawWelcome(const String &setupSsid) {
  tft.fillScreen(COLOR_BG);
  drawHeader("ACE DRY BOX MONITOR");
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.setTextSize(2);
  tft.drawString("Welcome", 18, 76, 4);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED, COLOR_BG);
  tft.drawString("Wi-Fi setup", 18, 142, 4);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.drawString("1. Join this network:", 18, 182, 2);
  tft.setTextColor(COLOR_WARN, COLOR_BG);
  tft.drawString(setupSsid, 18, 207, 2);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.drawString("2. Open 192.168.4.1", 18, 240, 2);
  tft.drawFastHLine(18, 278, 284, COLOR_MUTED);
  tft.setTextColor(COLOR_MUTED, COLOR_BG);
  tft.drawString("After setup, open:", 18, 302, 2);
  tft.setTextColor(COLOR_GOOD, COLOR_BG);
  tft.drawString("http://" + hostname + ".local", 18, 332, 4);
  tft.setTextColor(COLOR_MUTED, COLOR_BG);
  tft.drawString("Choose Wi-Fi or offline mode", 18, 378, 2);
  tft.fillRoundRect(8, 410, 148, 54, 6, 0x04A8);
  tft.fillRoundRect(164, 410, 148, 54, 6, 0x3186);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("USE WIFI", 82, 437, 2);
  tft.drawString("SKIP WIFI", 238, 437, 2);
  tft.setTextDatum(TL_DATUM);
}

uint16_t statusColor(float humidity) {
  if (humidity <= goodLimitRh) return COLOR_GOOD;
  if (humidity < warningLimitRh) return COLOR_WARN;
  return COLOR_BAD;
}

const char *statusText(float humidity) {
  if (humidity <= goodLimitRh) return "DRY";
  if (humidity < warningLimitRh) return "CHECK";
  return "HUMID";
}

bool online(const NodeState &node, uint32_t now) {
  return node.received && now - node.receivedMs <= OFFLINE_AFTER_MS;
}

void recordHistory(uint8_t slot, const DryBoxProtocol::SensorPacket &packet, uint32_t packetTime) {
  if (!(packet.flags & DryBoxProtocol::FLAG_SENSOR_OK) || !isfinite(packet.temperatureC) ||
      !isfinite(packet.humidityRh)) return;
  NodeHistory &history = nodeHistory[slot];
  const uint32_t bucket = packetTime / HISTORY_BUCKET_MS;
  HistorySample sample{};
  sample.bucket = bucket;
  sample.temperatureCenti = static_cast<int16_t>(constrain(lroundf(packet.temperatureC * 100.0f), -32768L, 32767L));
  sample.humidityCenti = static_cast<uint16_t>(constrain(lroundf(packet.humidityRh * 100.0f), 0L, 10000L));
  if (history.count && history.lastBucket == bucket) {
    const uint16_t latest = (history.head + HISTORY_BUCKETS - 1) % HISTORY_BUCKETS;
    history.samples[latest] = sample;
    return;
  }
  history.samples[history.head] = sample;
  history.head = (history.head + 1) % HISTORY_BUCKETS;
  if (history.count < HISTORY_BUCKETS) ++history.count;
  history.lastBucket = bucket;
}

void drawHistoryPlot(const NodeHistory &history, bool temperature, int16_t top, uint16_t color) {
  constexpr int16_t left = 31;
  constexpr int16_t right = 310;
  constexpr int16_t plotHeight = 99;
  const int16_t plotTop = top + 18;
  const int16_t bottom = plotTop + plotHeight;
  tft.drawRect(left, plotTop, right - left + 1, plotHeight + 1, COLOR_MUTED);
  tft.setTextColor(color, COLOR_BG);
  tft.drawString(temperature ? "TEMP 24H" : "HUMIDITY 24H", left, top, 2);
  tft.setTextColor(COLOR_MUTED, COLOR_BG);
  tft.drawString("-24h", left, bottom + 3, 1);
  tft.setTextDatum(TR_DATUM);
  tft.drawString("NOW", right, bottom + 3, 1);
  tft.setTextDatum(TL_DATUM);
  if (!history.count) {
    tft.setTextDatum(MC_DATUM);
    tft.drawString("COLLECTING DATA", (left + right) / 2, (plotTop + bottom) / 2, 2);
    tft.setTextDatum(TL_DATUM);
    return;
  }

  float minimum = 10000.0f;
  float maximum = -10000.0f;
  uint32_t newestBucket = 0;
  for (uint16_t n = 0; n < history.count; ++n) {
    const uint16_t i = (history.head + HISTORY_BUCKETS - history.count + n) % HISTORY_BUCKETS;
    const HistorySample &sample = history.samples[i];
    const float value = temperature ? sample.temperatureCenti / 100.0f : sample.humidityCenti / 100.0f;
    minimum = min(minimum, value);
    maximum = max(maximum, value);
    newestBucket = max(newestBucket, sample.bucket);
  }
  float padding = max((maximum - minimum) * 0.12f, temperature ? 0.5f : 2.0f);
  minimum -= padding;
  maximum += padding;
  char label[14];
  snprintf(label, sizeof(label), temperature ? "%.1fC" : "%.0f%%", maximum);
  tft.setTextColor(COLOR_MUTED, COLOR_BG);
  tft.setTextDatum(TR_DATUM);
  tft.drawString(label, left - 3, plotTop, 1);
  snprintf(label, sizeof(label), temperature ? "%.1fC" : "%.0f%%", minimum);
  tft.drawString(label, left - 3, bottom - 7, 1);
  tft.setTextDatum(TL_DATUM);

  bool havePrevious = false;
  int16_t previousX = 0;
  int16_t previousY = 0;
  uint32_t previousBucket = 0;
  for (uint16_t n = 0; n < history.count; ++n) {
    const uint16_t i = (history.head + HISTORY_BUCKETS - history.count + n) % HISTORY_BUCKETS;
    const HistorySample &sample = history.samples[i];
    const uint32_t age = newestBucket - sample.bucket;
    if (age >= HISTORY_BUCKETS) continue;
    const int16_t x = right - static_cast<int16_t>((age * (right - left)) / (HISTORY_BUCKETS - 1));
    const float value = temperature ? sample.temperatureCenti / 100.0f : sample.humidityCenti / 100.0f;
    const int16_t y = bottom - static_cast<int16_t>(((value - minimum) * plotHeight) / (maximum - minimum));
    if (havePrevious && sample.bucket - previousBucket <= 2) tft.drawLine(previousX, previousY, x, y, color);
    tft.fillCircle(x, y, 1, color);
    previousX = x;
    previousY = y;
    previousBucket = sample.bucket;
    havePrevious = true;
  }
}

void fillButton(int16_t x, int16_t y, int16_t w, int16_t h, const char *label, uint16_t color) {
  tft.fillRoundRect(x, y, w, h, 7, color);
  tft.drawRoundRect(x, y, w, h, 7, TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, color);
  tft.drawString(label, x + w / 2, y + h / 2, 2);
  tft.setTextDatum(TL_DATUM);
}

void drawHeader(const char *title, const char *right) {
  tft.fillRect(0, 0, SCREEN_WIDTH, 52, COLOR_HEADER);
  tft.setTextColor(TFT_WHITE, COLOR_HEADER);
  tft.setTextDatum(ML_DATUM);
  tft.drawString(title, 12, 19, 4);
  if (right) {
    tft.setTextDatum(MR_DATUM);
    tft.drawString(right, 308, 26, 2);
  }
  tft.setTextDatum(TL_DATUM);
}

void drawOverview() {
  NodeState snapshot[DryBoxProtocol::NODE_COUNT];
  portENTER_CRITICAL(&nodeMux);
  memcpy(snapshot, nodes, sizeof(snapshot));
  portEXIT_CRITICAL(&nodeMux);

  const uint32_t now = millis();
  tft.fillScreen(COLOR_BG);
  char countText[20];
  uint8_t onlineCount = 0;
  for (const auto &node : snapshot) onlineCount += online(node, now) ? 1 : 0;
  snprintf(countText, sizeof(countText), "%u/10 ONLINE", onlineCount);
  drawHeader("DRY BOXES", countText);
  tft.setTextColor(WiFi.status() == WL_CONNECTED ? COLOR_GOOD : COLOR_WARN, COLOR_HEADER);
  const String wifiStatus = WiFi.status() == WL_CONNECTED
                                ? "WiFi " + WiFi.localIP().toString()
                                : (wifiSkippedAtStartup ? "WiFi offline mode" : "WiFi disconnected");
  tft.drawString(wifiStatus, 12, 39, 1);

  constexpr int16_t firstY = 60;
  constexpr int16_t rowH = 70;
  const uint8_t firstNode = overviewPage * 5;
  for (uint8_t row = 0; row < 5; ++row) {
    const uint8_t i = firstNode + row;
    const int16_t y = firstY + row * rowH;
    const bool isOnline = online(snapshot[i], now);
    const bool sensorOk = isOnline && (snapshot[i].packet.flags & DryBoxProtocol::FLAG_SENSOR_OK);
    uint16_t color = COLOR_MUTED;
    if (sensorOk) color = statusColor(snapshot[i].packet.humidityRh);

    tft.fillRoundRect(7, y, 306, 62, 7, COLOR_PANEL);
    tft.fillRoundRect(7, y, 9, 62, 5, color);
    tft.setTextColor(TFT_WHITE, COLOR_PANEL);
    tft.drawString(nodeName(i), 24, y + 8, 4);

    if (sensorOk) {
      char value[32];
      const float temperatureF = snapshot[i].packet.temperatureC * 9.0f / 5.0f + 32.0f;
      snprintf(value, sizeof(value), "%.1f C / %.1f F", snapshot[i].packet.temperatureC, temperatureF);
      tft.setTextColor(0xDFFF, COLOR_PANEL);
      tft.drawString(value, 24, y + 38, 2);
      snprintf(value, sizeof(value), "%.1f%%", snapshot[i].packet.humidityRh);
      tft.setTextDatum(MR_DATUM);
      tft.setTextColor(color, COLOR_PANEL);
      tft.drawString(value, 298, y + 24, 4);
      tft.setTextDatum(TL_DATUM);
      tft.setTextColor(color, COLOR_PANEL);
      tft.drawString(statusText(snapshot[i].packet.humidityRh), 196, y + 43, 2);
    } else {
      tft.setTextDatum(MR_DATUM);
      tft.setTextColor(COLOR_MUTED, COLOR_PANEL);
      tft.drawString(isOnline ? "SENSOR ERROR" : "OFFLINE", 298, y + 29, 2);
      tft.setTextDatum(TL_DATUM);
    }
  }

  fillButton(5, 418, 66, 51, "1-5", 0x3186);
  fillButton(75, 418, 66, 51, "6-10", 0x3186);
  fillButton(145, 418, 82, 51, "WEATHER", 0x056B);
  fillButton(231, 418, 84, 51, "SET", 0x3186);
}

void drawWeather() {
  tft.fillScreen(COLOR_BG);
  char dateTime[24] = "TIME SYNCING";
  const time_t now = time(nullptr);
  tm localTime{};
  if (now > 100000 && localtime_r(&now, &localTime)) {
    strftime(dateTime, sizeof(dateTime), "%b %d %I:%M %p", &localTime);
  }
  drawHeader(weatherLocation.c_str(), dateTime);
  const char unit = weatherImperial ? 'F' : 'C';
  if (!weatherApiKey.length()) {
    tft.setTextColor(COLOR_WARN, COLOR_BG);
    tft.drawString("OPENWEATHER SETUP NEEDED", 18, 84, 2);
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    tft.drawString("Open the local web dashboard", 18, 120, 2);
    tft.drawString("and enter API/location settings.", 18, 146, 2);
  } else if (!weatherData.current.valid) {
    tft.setTextColor(COLOR_WARN, COLOR_BG);
    tft.drawString("WAITING FOR WEATHER", 18, 84, 4);
    tft.setTextColor(COLOR_MUTED, COLOR_BG);
    tft.drawString(weatherError, 18, 125, 2);
  } else {
    char value[40];
    tft.setTextColor(TFT_WHITE, COLOR_BG);
    snprintf(value, sizeof(value), "%.1f", weatherData.current.temperature);
    tft.drawString(value, 14, 68, 7);
    const int16_t unitX = 14 + tft.textWidth(value, 7) + 7;
    tft.drawCircle(unitX + 4, 77, 4, TFT_WHITE);
    tft.drawString(String(unit), unitX + 12, 70, 4);
    tft.setTextColor(COLOR_MUTED, COLOR_BG);
    tft.drawString(weatherData.current.description, 16, 120, 4);
    snprintf(value, sizeof(value), "Feels %.0f%c   RH %.0f%%   Wind %.0f %s", weatherData.current.feelsLike, unit,
             weatherData.current.humidity, weatherData.current.windSpeed, weatherImperial ? "mph" : "m/s");
    tft.drawString(value, 16, 153, 2);
  }

  tft.fillRoundRect(8, 184, 304, 72, 7, COLOR_PANEL);
  tft.setTextColor(COLOR_GOOD, COLOR_PANEL);
  tft.drawString("ROOM - LOCAL SENSOR", 18, 194, 2);
  if (localReading.bmpOk) {
    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(COLOR_MUTED, COLOR_PANEL);
    tft.drawString(String(localReading.pressureHpa, 0) + " hPa", 298, 204, 2);
    tft.setTextDatum(TL_DATUM);
  }
  tft.setTextColor(TFT_WHITE, COLOR_PANEL);
  if (localReading.ahtOk) {
    const float roomTempF = localReading.temperatureC * 9.0f / 5.0f + 32.0f;
    char roomValue[48];
    snprintf(roomValue, sizeof(roomValue), "%.1f C / %.1f F   %.1f%%", localReading.temperatureC, roomTempF,
             localReading.humidityRh);
    tft.drawString(roomValue, 18, 220, 4);
  } else {
    tft.drawString("Sensor offline", 18, 220, 4);
  }

  for (uint8_t i = 0; i < 4; ++i) {
    const int16_t x = 5 + i * 79;
    tft.fillRoundRect(x, 268, 74, 132, 6, COLOR_PANEL);
    if (!weatherData.forecast[i].valid) continue;
    tm day{};
    localtime_r(&weatherData.forecast[i].date, &day);
    char label[8];
    strftime(label, sizeof(label), "%a", &day);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(COLOR_WARN, COLOR_PANEL);
    tft.drawString(label, x + 37, 278, 2);
    char forecastValue[20];
    snprintf(forecastValue, sizeof(forecastValue), "%.0f/%.0f", weatherData.forecast[i].high, weatherData.forecast[i].low);
    tft.setTextColor(TFT_WHITE, COLOR_PANEL);
    tft.drawString(forecastValue, x + 37, 307, 4);
    snprintf(forecastValue, sizeof(forecastValue), "Rain %.0f%%", weatherData.forecast[i].precipitation * 100.0f);
    tft.setTextColor(COLOR_MUTED, COLOR_PANEL);
    tft.drawString(forecastValue, x + 37, 348, 1);
    tft.setTextDatum(TL_DATUM);
  }
  fillButton(8, 418, 96, 51, "BACK", 0x3186);
  fillButton(112, 418, 96, 51, weatherImperial ? "SHOW C" : "SHOW F", 0x04A8);
  fillButton(216, 418, 96, 51, "REFRESH", 0x056B);
}

void drawDetail(uint8_t index) {
  NodeState node;
  NodeHistory history;
  portENTER_CRITICAL(&nodeMux);
  node = nodes[index];
  history = nodeHistory[index];
  portEXIT_CRITICAL(&nodeMux);
  const uint32_t now = millis();

  tft.fillScreen(COLOR_BG);
  drawHeader(nodeName(index), online(node, now) ? "ONLINE" : "OFFLINE");
  fillButton(8, 420, 304, 50, "BACK", 0x3186);

  if (!online(node, now)) {
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(COLOR_MUTED, COLOR_BG);
    tft.drawString("NO RECENT DATA", 160, 220, 4);
    tft.setTextDatum(TL_DATUM);
    return;
  }
  if (!(node.packet.flags & DryBoxProtocol::FLAG_SENSOR_OK)) {
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(COLOR_BAD, COLOR_BG);
    tft.drawString("AHT20 ERROR", 160, 220, 4);
    tft.setTextDatum(TL_DATUM);
    return;
  }

  const uint16_t color = statusColor(node.packet.humidityRh);
  char value[40];
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  const float temperatureF = node.packet.temperatureC * 9.0f / 5.0f + 32.0f;
  snprintf(value, sizeof(value), "%.1f C / %.1f F", node.packet.temperatureC, temperatureF);
  tft.drawString(value, 10, 61, 4);
  tft.setTextColor(color, COLOR_BG);
  snprintf(value, sizeof(value), "%.1f%% RH", node.packet.humidityRh);
  tft.setTextDatum(TR_DATUM);
  tft.drawString(value, 310, 64, 2);
  tft.drawString(statusText(node.packet.humidityRh), 310, 80, 2);
  tft.setTextDatum(TL_DATUM);
  if (node.packet.flags & DryBoxProtocol::FLAG_PRESSURE_OK) {
    tft.setTextColor(COLOR_MUTED, COLOR_BG);
    snprintf(value, sizeof(value), "%.1f hPa", node.packet.pressureHpa);
    tft.drawString(value, 10, 87, 1);
  }
  drawHistoryPlot(history, true, 98, COLOR_WARN);
  drawHistoryPlot(history, false, 250, 0x07FF);
  tft.setTextColor(COLOR_MUTED, COLOR_BG);
  snprintf(value, sizeof(value), "Packet %lu  Age %lus", static_cast<unsigned long>(node.packet.sequence),
           static_cast<unsigned long>((now - node.receivedMs) / 1000));
  tft.drawString(value, 31, 402, 1);
  tft.setTextDatum(TL_DATUM);
}

void drawSettings() {
  tft.fillScreen(COLOR_BG);
  drawHeader("SETTINGS");
  tft.setTextColor(TFT_WHITE, COLOR_BG);
  tft.drawString("DRY LIMIT", 18, 91, 4);
  tft.drawString("HUMID LIMIT", 18, 203, 4);

  char value[16];
  snprintf(value, sizeof(value), "%.0f%%", goodLimitRh);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COLOR_GOOD, COLOR_BG);
  tft.drawString(value, 160, 150, 6);
  snprintf(value, sizeof(value), "%.0f%%", warningLimitRh);
  tft.setTextColor(COLOR_BAD, COLOR_BG);
  tft.drawString(value, 160, 262, 6);
  tft.setTextDatum(TL_DATUM);

  fillButton(10, 130, 54, 48, "-", 0x3186);
  fillButton(256, 130, 54, 48, "+", 0x3186);
  fillButton(10, 242, 54, 48, "-", 0x3186);
  fillButton(256, 242, 54, 48, "+", 0x3186);
  fillButton(8, 332, 148, 56, "PAIR NODES", 0x3186);
  fillButton(164, 332, 148, 56, "CAL TOUCH", 0x3186);
  fillButton(8, 420, 304, 50, "SAVE & BACK", 0x04A8);
}

void drawPairing() {
  tft.fillScreen(COLOR_BG);
  drawHeader("PAIR NODES", pairingActive ? "LISTENING" : "SELECT SLOT");
  const uint8_t firstSlot = overviewPage * 5;
  for (uint8_t row = 0; row < 5; ++row) {
    const uint8_t i = firstSlot + row;
    const int16_t y = 58 + row * 52;
    const uint16_t fill = i == selectedPairSlot ? 0x3186 : COLOR_PANEL;
    tft.fillRoundRect(8, y, 304, 46, 6, fill);
    tft.setTextColor(TFT_WHITE, fill);
    tft.drawString(nodeName(i), 20, y + 12, 4);
    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(slotPaired[i] ? COLOR_GOOD : COLOR_MUTED, fill);
    tft.drawString(slotPaired[i] ? "PAIRED" : "EMPTY", 298, y + 23, 2);
    tft.setTextDatum(TL_DATUM);
  }
  fillButton(104, 313, 112, 30, overviewPage == 0 ? "SHOW 6-10" : "SHOW 1-5", 0x3186);
  fillButton(8, 350, 148, 54, pairingActive ? "CANCEL PAIR" : "PAIR", 0x04A8);
  fillButton(164, 350, 148, 54, "UNPAIR", 0xA800);
  fillButton(8, 420, 304, 50, "BACK", 0x3186);
}

void redraw() {
  if (weatherScreen) drawWeather();
  else if (pairingScreen) drawPairing();
  else if (settingsScreen) drawSettings();
  else if (detailNode >= 0) drawDetail(detailNode);
  else drawOverview();
}

bool readTouch(int16_t &x, int16_t &y) {
  if (!touch.touched() || millis() - lastTouchMs < 180) return false;
  TS_Point raw = touch.getPoint();
  lastTouchMs = millis();
  x = constrain(map(raw.x, touchMinX, touchMaxX, 0, SCREEN_WIDTH - 1), 0, SCREEN_WIDTH - 1);
  y = constrain(map(raw.y, touchMinY, touchMaxY, 0, SCREEN_HEIGHT - 1), 0, SCREEN_HEIGHT - 1);
  return true;
}

bool skipWifiSelected() {
  const uint32_t choiceStartedMs = millis();
  uint8_t previousSeconds = UINT8_MAX;
  while (millis() - choiceStartedMs < WIFI_CHOICE_TIMEOUT_MS) {
    const uint32_t remainingMs = WIFI_CHOICE_TIMEOUT_MS - (millis() - choiceStartedMs);
    const uint8_t remainingSeconds = (remainingMs + 999) / 1000;
    if (remainingSeconds != previousSeconds) {
      previousSeconds = remainingSeconds;
      tft.fillRect(8, 465, 304, 15, COLOR_BG);
      tft.setTextDatum(BC_DATUM);
      tft.setTextColor(COLOR_MUTED, COLOR_BG);
      tft.drawString("Auto-start Wi-Fi in " + String(remainingSeconds) + "s", SCREEN_WIDTH / 2, 479, 1);
      tft.setTextDatum(TL_DATUM);
    }
    int16_t x, y;
    if (readTouch(x, y) && y >= 400) {
      while (touch.touched()) delay(10);
      return x >= SCREEN_WIDTH / 2;
    }
    delay(20);
  }
  return false;
}

void saveThresholds() {
  preferences.begin("drybox", false);
  preferences.putFloat("goodRh", goodLimitRh);
  preferences.putFloat("warnRh", warningLimitRh);
  preferences.end();
}

void savePairedSlot(uint8_t slot) {
  char key[8];
  snprintf(key, sizeof(key), "mac%u", slot + 1);
  preferences.begin("drybox", false);
  if (slotPaired[slot]) preferences.putBytes(key, pairedMacs[slot], 6);
  else preferences.remove(key);
  preferences.end();
}

void addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void unpairSlot(uint8_t slot) {
  if (!slotPaired[slot]) return;
  DryBoxProtocol::UnpairCommand command{};
  command.magic = DryBoxProtocol::UNPAIR_MAGIC;
  command.version = DryBoxProtocol::VERSION;
  command.nodeId = slot + 1;
  command.checksum = DryBoxProtocol::messageChecksum(command);
  esp_now_send(pairedMacs[slot], reinterpret_cast<const uint8_t *>(&command), sizeof(command));
  delay(20);
  esp_now_del_peer(pairedMacs[slot]);
  memset(pairedMacs[slot], 0, 6);
  slotPaired[slot] = false;
  portENTER_CRITICAL(&nodeMux);
  nodes[slot] = NodeState{};
  portEXIT_CRITICAL(&nodeMux);
  savePairedSlot(slot);
}

String setupNetworkName() {
  char suffix[7];
  snprintf(suffix, sizeof(suffix), "%06llX", static_cast<unsigned long long>(ESP.getEfuseMac() & 0xFFFFFFULL));
  return "DryBoxMonitor-Setup-" + String(suffix);
}

bool connectWifi() {
  preferences.begin("drybox", true);
  hostname = sanitizeHostname(preferences.getString("hostname", "drybox-monitor"));
  weatherApiKey = preferences.getString("weatherKey", "");
  weatherLatitude = preferences.getString("weatherLat", weatherLatitude);
  weatherLongitude = preferences.getString("weatherLon", weatherLongitude);
  weatherLocation = preferences.getString("weatherPlace", weatherLocation);
  weatherTimezone = preferences.getString("weatherTz", weatherTimezone);
  weatherImperial = preferences.getBool("weatherImp", true);
  preferences.end();

  const String setupSsid = setupNetworkName();
  drawWelcome(setupSsid);
  if (skipWifiSelected()) {
    wifiSkippedAtStartup = true;
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false);
    WiFi.disconnect(false, false);
    Serial.println("[wifi] SKIPPED for this boot; ESP-NOW offline mode enabled");
    return false;
  }
  char hostnameBuffer[33], apiBuffer[65], latitudeBuffer[17], longitudeBuffer[17], locationBuffer[33], timezoneBuffer[65];
  hostname.toCharArray(hostnameBuffer, sizeof(hostnameBuffer));
  weatherApiKey.toCharArray(apiBuffer, sizeof(apiBuffer));
  weatherLatitude.toCharArray(latitudeBuffer, sizeof(latitudeBuffer));
  weatherLongitude.toCharArray(longitudeBuffer, sizeof(longitudeBuffer));
  weatherLocation.toCharArray(locationBuffer, sizeof(locationBuffer));
  weatherTimezone.toCharArray(timezoneBuffer, sizeof(timezoneBuffer));
  WiFiManager manager;
  WiFiManagerParameter hostnameParameter("host", "Device hostname (without .local)", hostnameBuffer, 32);
  WiFiManagerParameter apiParameter("weatherKey", "OpenWeather API key", apiBuffer, 64, "type='password'");
  WiFiManagerParameter latitudeParameter("weatherLat", "Weather latitude", latitudeBuffer, 16);
  WiFiManagerParameter longitudeParameter("weatherLon", "Weather longitude", longitudeBuffer, 16);
  WiFiManagerParameter locationParameter("weatherPlace", "Weather location name", locationBuffer, 32);
  WiFiManagerParameter timezoneParameter("weatherTz", "POSIX timezone", timezoneBuffer, 64);
  manager.addParameter(&hostnameParameter);
  manager.addParameter(&apiParameter);
  manager.addParameter(&latitudeParameter);
  manager.addParameter(&longitudeParameter);
  manager.addParameter(&locationParameter);
  manager.addParameter(&timezoneParameter);
  manager.setHostname(hostname.c_str());
  manager.setConnectTimeout(20);
  manager.setConfigPortalTimeout(300);
  const bool connected = manager.autoConnect(setupSsid.c_str());
  WiFi.setAutoReconnect(true);
  if (connected) {
    hostname = sanitizeHostname(hostnameParameter.getValue());
    weatherApiKey = apiParameter.getValue();
    weatherLatitude = latitudeParameter.getValue();
    weatherLongitude = longitudeParameter.getValue();
    weatherLocation = locationParameter.getValue();
    weatherTimezone = timezoneParameter.getValue();
    preferences.begin("drybox", false);
    preferences.putString("hostname", hostname);
    preferences.putString("weatherKey", weatherApiKey);
    preferences.putString("weatherLat", weatherLatitude);
    preferences.putString("weatherLon", weatherLongitude);
    preferences.putString("weatherPlace", weatherLocation);
    preferences.putString("weatherTz", weatherTimezone);
    preferences.end();
    WiFi.setHostname(hostname.c_str());
  }
  return connected;
}

void sendJsonStatus() {
  NodeState snapshot[DryBoxProtocol::NODE_COUNT];
  portENTER_CRITICAL(&nodeMux);
  memcpy(snapshot, nodes, sizeof(snapshot));
  portEXIT_CRITICAL(&nodeMux);
  const uint32_t now = millis();
  String json;
  json.reserve(1800);
  json = "{\"hostname\":\"" + hostname + "\",\"ip\":\"" + WiFi.localIP().toString() +
         "\",\"channel\":" + String(WiFi.channel()) + ",\"goodLimitRh\":" + String(goodLimitRh, 0) +
         ",\"warningLimitRh\":" + String(warningLimitRh, 0) + ",\"nodes\":[";
  for (uint8_t i = 0; i < DryBoxProtocol::NODE_COUNT; ++i) {
    if (i) json += ',';
    const bool isOnline = online(snapshot[i], now);
    const bool sensorOk = isOnline && (snapshot[i].packet.flags & DryBoxProtocol::FLAG_SENSOR_OK);
    json += "{\"id\":" + String(i + 1) + ",\"name\":\"" + nodeName(i) + "\",\"paired\":" +
            String(slotPaired[i] ? "true" : "false") + ",\"online\":" + String(isOnline ? "true" : "false") +
            ",\"sensorOk\":" + String(sensorOk ? "true" : "false");
    if (sensorOk) {
      json += ",\"temperatureC\":" + String(snapshot[i].packet.temperatureC, 1) +
              ",\"humidityRh\":" + String(snapshot[i].packet.humidityRh, 1) +
              ",\"pressureHpa\":" + String(snapshot[i].packet.pressureHpa, 1) +
              ",\"sequence\":" + String(snapshot[i].packet.sequence) +
              ",\"ageSeconds\":" + String((now - snapshot[i].receivedMs) / 1000UL);
    }
    json += '}';
  }
  json += "]}";
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "application/json", json);
}

void sendWebDashboard() {
  String page = F(R"HTML(<!doctype html><html><head><meta name=viewport content="width=device-width,initial-scale=1">
<title>Dry Box Monitor</title><style>
body{font:16px system-ui;background:#07131b;color:#eef7fa;max-width:1050px;margin:auto;padding:18px}h1{margin-bottom:4px}.sub{color:#9fb2bd;margin-top:0}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:12px}.card{background:#12242e;border-left:7px solid #60747e;border-radius:9px;padding:14px}.good{border-color:#20c970}.warn{border-color:#ffab23}.bad{border-color:#ef4444}.off{opacity:.65}.value{font-size:29px;font-weight:700}.meta{color:#9fb2bd}button,input{padding:10px;margin:5px;border:0;border-radius:6px}button{background:#237da0;color:white;font-weight:700}.danger{background:#b33131}.panel{background:#12242e;padding:14px;border-radius:9px;margin-top:16px}</style></head><body>
<h1>ACE Dry Box Monitor</h1><p class=sub id=connection>Loading...</p><div class=grid id=nodes></div>
<div class=panel><h2>Node setup</h2><p>Select an empty slot on the touchscreen, or start pairing here:</p><span id=pairButtons></span></div>
<div class=panel><h2>Weather setup</h2><form method=post action=/weather-settings><label>OpenWeather API key </label><input type=password name=key placeholder="Leave blank to keep saved key"><br><label>Location </label><input name=place value=")HTML");
  page += weatherLocation;
  page += F(R"HTML("><br><label>Latitude </label><input name=lat value=")HTML");
  page += weatherLatitude;
  page += F(R"HTML("><label> Longitude </label><input name=lon value=")HTML");
  page += weatherLongitude;
  page += F(R"HTML("><br><label>POSIX timezone </label><input name=tz size=35 value=")HTML");
  page += weatherTimezone;
  page += "\"><br><label>AHT20 display temperature offset (&deg;C) </label><input type=number name=tempOffset min=-20 max=20 step=0.1 value=\"" +
          String(localTemperatureOffsetC, 1) + "\"><br><label><input type=checkbox name=imperial" +
          String(weatherImperial ? " checked" : "") + "> Fahrenheit / mph</label><br><button>Save weather</button></form></div>";
  page += F(R"HTML(<div class=panel><h2>Controller</h2><form method=post action=/hostname><label>Hostname </label><input name=host maxlength=32 pattern="[a-zA-Z0-9-]+" value=")HTML");
  page += hostname;
  page += F(R"HTML("><button>Save and restart</button></form><form method=post action=/wifi-reset onsubmit="return confirm('Erase Wi-Fi and restart setup?')"><button class=danger>Change Wi-Fi</button></form></div>
<script>
async function post(url){await fetch(url,{method:'POST'});setTimeout(load,300)}
async function load(){try{let d=await(await fetch('/api/status',{cache:'no-store'})).json();connection.textContent='http://'+d.hostname+'.local  |  '+d.ip+'  |  Wi-Fi channel '+d.channel;
nodes.innerHTML=d.nodes.map(n=>{let cls=!n.online?'off':!n.sensorOk?'bad':n.humidityRh<=d.goodLimitRh?'good':n.humidityRh<d.warningLimitRh?'warn':'bad';let f=n.temperatureC*9/5+32;let v=n.sensorOk?`<div class=value>${n.temperatureC.toFixed(1)} C / ${f.toFixed(1)} F &nbsp; ${n.humidityRh.toFixed(1)}% RH</div><div class=meta>Packet ${n.sequence}, ${n.ageSeconds}s ago</div>`:`<div class=value>${n.paired?(n.online?'SENSOR ERROR':'OFFLINE'):'NOT PAIRED'}</div>`;return `<div class="card ${cls}"><h2>${n.name}</h2>${v}${n.paired?`<button class=danger onclick="post('/unpair?slot=${n.id}')">Unpair</button>`:''}</div>`}).join('');
pairButtons.innerHTML=d.nodes.filter(n=>!n.paired).map(n=>`<button onclick="post('/pair?slot=${n.id}')">Pair ${n.name}</button>`).join('')||'All slots are paired.';}catch(e){connection.textContent='Controller unavailable';}}load();setInterval(load,3000);
</script></body></html>)HTML");
  webServer.send(200, "text/html", page);
}

void startWebServer() {
  if (WiFi.status() != WL_CONNECTED) return;
  webServer.on("/", HTTP_GET, sendWebDashboard);
  webServer.on("/api/status", HTTP_GET, sendJsonStatus);
  webServer.on("/pair", HTTP_POST, []() {
    const int slot = webServer.arg("slot").toInt() - 1;
    if (slot < 0 || slot >= DryBoxProtocol::NODE_COUNT || slotPaired[slot]) {
      webServer.send(409, "text/plain", "Slot is invalid or already paired");
      return;
    }
    selectedPairSlot = slot;
    overviewPage = slot / 5;
    pairingActive = true;
    pairingEndsMs = millis() + 60000;
    Serial.printf("[pair] LISTENING from web slot=ACE %u controllerChannel=%u\n", selectedPairSlot + 1,
                  WiFi.channel());
    webServer.send(200, "text/plain", "Pairing enabled for 60 seconds");
  });
  webServer.on("/unpair", HTTP_POST, []() {
    const int slot = webServer.arg("slot").toInt() - 1;
    if (slot < 0 || slot >= DryBoxProtocol::NODE_COUNT) {
      webServer.send(400, "text/plain", "Invalid slot");
      return;
    }
    unpairSlot(slot);
    webServer.send(200, "text/plain", "Node unpaired");
  });
  webServer.on("/hostname", HTTP_POST, []() {
    hostname = sanitizeHostname(webServer.arg("host"));
    preferences.begin("drybox", false);
    preferences.putString("hostname", hostname);
    preferences.end();
    webServer.send(200, "text/html", "<h2>Saved. Restarting...</h2><p>Open http://" + hostname + ".local in about 20 seconds.</p>");
    delay(500);
    ESP.restart();
  });
  webServer.on("/weather-settings", HTTP_POST, []() {
    if (webServer.arg("key").length()) weatherApiKey = webServer.arg("key");
    weatherLocation = webServer.arg("place");
    weatherLatitude = webServer.arg("lat");
    weatherLongitude = webServer.arg("lon");
    weatherTimezone = webServer.arg("tz");
    weatherImperial = webServer.hasArg("imperial");
    localTemperatureOffsetC = constrain(webServer.arg("tempOffset").toFloat(), -20.0f, 20.0f);
    preferences.begin("drybox", false);
    preferences.putString("weatherKey", weatherApiKey);
    preferences.putString("weatherPlace", weatherLocation);
    preferences.putString("weatherLat", weatherLatitude);
    preferences.putString("weatherLon", weatherLongitude);
    preferences.putString("weatherTz", weatherTimezone);
    preferences.putBool("weatherImp", weatherImperial);
    preferences.putFloat("localTempOff", localTemperatureOffsetC);
    preferences.end();
    readLocalSensor();
    setenv("TZ", weatherTimezone.c_str(), 1);
    tzset();
    lastWeatherAttemptMs = 0;
    webServer.sendHeader("Location", "/");
    webServer.send(302, "text/plain", "");
  });
  webServer.on("/wifi-reset", HTTP_POST, []() {
    webServer.send(200, "text/html", "<h2>Wi-Fi erased. Restarting setup...</h2>");
    WiFiManager manager;
    manager.resetSettings();
    delay(500);
    ESP.restart();
  });
  webServer.onNotFound([]() {
    webServer.sendHeader("Location", "/");
    webServer.send(302, "text/plain", "");
  });
  webServer.begin();
  webServerStarted = true;
  if (MDNS.begin(hostname.c_str())) MDNS.addService("http", "tcp", 80);
  Serial.println("[web] http://" + hostname + ".local");
  Serial.println("[web] http://" + WiFi.localIP().toString());
}

void handleWifiRecovery(uint32_t now) {
  if (wifiSkippedAtStartup) return;
  if (WiFi.status() == WL_CONNECTED) {
    const uint8_t currentChannel = WiFi.channel();
    if (!wifiWasConnected) {
      const uint32_t outageSeconds = wifiDisconnectedSinceMs ? (now - wifiDisconnectedSinceMs) / 1000UL : 0;
      Serial.printf("[wifi] RECOVERED after %lus ssid=%s rssi=%ddBm channel=%u ip=%s\n",
                    static_cast<unsigned long>(outageSeconds), WiFi.SSID().c_str(), WiFi.RSSI(), currentChannel,
                    WiFi.localIP().toString().c_str());
      wifiWasConnected = true;
      wifiDisconnectedSinceMs = 0;
      lastWifiReconnectAttemptMs = 0;
      wifiReconnectAttempts = 0;
      if (!webServerStarted) {
        startWebServer();
      } else {
        MDNS.end();
        if (MDNS.begin(hostname.c_str())) MDNS.addService("http", "tcp", 80);
      }
      setenv("TZ", weatherTimezone.c_str(), 1);
      tzset();
      configTzTime(weatherTimezone.c_str(), "pool.ntp.org", "time.nist.gov", "time.google.com");
      lastWeatherAttemptMs = 0;
      packetPendingRedraw = true;
    }
    if (lastConnectedWifiChannel != currentChannel) {
      Serial.printf("[wifi] channel changed %u -> %u; ESP-NOW now follows channel %u\n",
                    lastConnectedWifiChannel, currentChannel, currentChannel);
      lastConnectedWifiChannel = currentChannel;
      lastPairBeaconMs = 0;
    }
    return;
  }

  if (wifiWasConnected) {
    wifiWasConnected = false;
    wifiDisconnectedSinceMs = now;
    wifiReconnectAttempts = 0;
    Serial.printf("[wifi] LOST status=%d; automatic recovery started\n", static_cast<int>(WiFi.status()));
    packetPendingRedraw = true;
  } else if (wifiDisconnectedSinceMs == 0) {
    wifiDisconnectedSinceMs = now;
  }

  if (lastWifiReconnectAttemptMs != 0 && now - lastWifiReconnectAttemptMs < WIFI_RECONNECT_INTERVAL_MS) return;
  lastWifiReconnectAttemptMs = now;
  ++wifiReconnectAttempts;
  if (wifiReconnectAttempts >= WIFI_RECONNECTS_BEFORE_RESTART) {
    Serial.printf("[wifi] restart connection attempt=%u status=%d\n", wifiReconnectAttempts,
                  static_cast<int>(WiFi.status()));
    WiFi.disconnect(false, false);
    WiFi.begin();
    wifiReconnectAttempts = 0;
  } else {
    const bool queued = WiFi.reconnect();
    Serial.printf("[wifi] reconnect attempt=%u queued=%s status=%d\n", wifiReconnectAttempts,
                  queued ? "yes" : "no", static_cast<int>(WiFi.status()));
  }
}

void handleTouch() {
  int16_t x, y;
  if (!readTouch(x, y)) return;

  if (weatherScreen) {
    if (y >= 410 && x < 108) {
      weatherScreen = false;
    } else if (y >= 410 && x < 212) {
      weatherImperial = !weatherImperial;
      preferences.begin("drybox", false);
      preferences.putBool("weatherImp", weatherImperial);
      preferences.end();
      weatherData = WeatherData{};
      lastWeatherAttemptMs = 0;
    } else if (y >= 410) {
      lastWeatherAttemptMs = 0;
    }
    redraw();
    return;
  }

  if (pairingScreen) {
    if (y >= 58 && y < 313) {
      selectedPairSlot = overviewPage * 5 + constrain((y - 58) / 52, 0, 4);
      pairingActive = false;
    } else if (y >= 313 && y < 347) {
      overviewPage = overviewPage == 0 ? 1 : 0;
      selectedPairSlot = overviewPage * 5;
      pairingActive = false;
    } else if (y >= 342 && y < 412 && x < 160) {
      if (slotPaired[selectedPairSlot]) {
        pairingActive = false;
      } else {
        pairingActive = !pairingActive;
        pairingEndsMs = millis() + 60000;
        Serial.printf("[pair] %s slot=ACE %u controllerChannel=%u\n", pairingActive ? "LISTENING" : "CANCELLED",
                      selectedPairSlot + 1, WiFi.channel());
      }
    } else if (y >= 342 && y < 412 && x >= 160) {
      pairingActive = false;
      unpairSlot(selectedPairSlot);
    } else if (y >= 412) {
      pairingActive = false;
      pairingScreen = false;
      settingsScreen = true;
    }
    redraw();
    return;
  }

  if (settingsScreen) {
    if (y >= 120 && y <= 188) {
      if (x < 90) goodLimitRh = max(10.0f, goodLimitRh - 1.0f);
      if (x > 230) goodLimitRh = min(warningLimitRh - 2.0f, goodLimitRh + 1.0f);
    } else if (y >= 232 && y <= 300) {
      if (x < 90) warningLimitRh = max(goodLimitRh + 2.0f, warningLimitRh - 1.0f);
      if (x > 230) warningLimitRh = min(80.0f, warningLimitRh + 1.0f);
    } else if (y >= 322 && y < 405) {
      if (x < 160) {
        settingsScreen = false;
        pairingScreen = true;
      } else {
        runTouchCalibration();
      }
    } else if (y >= 410) {
      saveThresholds();
      settingsScreen = false;
    }
    redraw();
    return;
  }

  if (detailNode >= 0) {
    if (y >= 410) {
      detailNode = -1;
      redraw();
    }
    return;
  }

  if (y >= 418) {
    if (x < 73) overviewPage = 0;
    else if (x < 143) overviewPage = 1;
    else if (x < 229) weatherScreen = true;
    else settingsScreen = true;
    redraw();
    return;
  }
  if (y >= 60 && y < 410) {
    const uint8_t selected = overviewPage * 5 + (y - 60) / 70;
    if (selected < DryBoxProtocol::NODE_COUNT) {
      detailNode = selected;
      redraw();
    }
  }
}

void onPacket(const uint8_t *sourceMac, const uint8_t *data, int length) {
  if (length == sizeof(DryBoxProtocol::PairRequest)) {
    DryBoxProtocol::PairRequest request;
    memcpy(&request, data, sizeof(request));
    if (!pairingActive || !DryBoxProtocol::valid(request)) return;
    if (slotPaired[selectedPairSlot] && memcmp(sourceMac, pairedMacs[selectedPairSlot], 6) != 0) return;
    uint8_t actualChannel = 1;
    wifi_second_chan_t secondaryChannel = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&actualChannel, &secondaryChannel);
    Serial.printf("[pair] request mac=%02X:%02X:%02X:%02X:%02X:%02X slot=ACE %u channel=%u nonce=%lu\n",
                  sourceMac[0], sourceMac[1], sourceMac[2], sourceMac[3], sourceMac[4], sourceMac[5],
                  selectedPairSlot + 1, actualChannel, static_cast<unsigned long>(request.nonce));
    portENTER_CRITICAL(&nodeMux);
    pendingPair.request = request;
    memcpy(pendingPair.sourceMac, sourceMac, 6);
    pendingPair.receivedChannel = actualChannel;
    pendingPair.pending = true;
    portEXIT_CRITICAL(&nodeMux);
    return;
  }
  if (length != sizeof(DryBoxProtocol::SensorPacket)) return;
  DryBoxProtocol::SensorPacket packet;
  memcpy(&packet, data, sizeof(packet));
  if (!DryBoxProtocol::valid(packet)) return;

  const uint8_t slot = packet.nodeId - 1;
  if (!slotPaired[slot] || memcmp(sourceMac, pairedMacs[slot], 6) != 0) return;
  if (pairingActive && slot == selectedPairSlot) {
    pairingActive = false;
    Serial.printf("[pair] ACE %u confirmed by first sensor packet\n", slot + 1);
  }

  const float packetTemperatureF = packet.temperatureC * 9.0f / 5.0f + 32.0f;
  Serial.printf("[receive] mac=%02X:%02X:%02X:%02X:%02X:%02X node=%u seq=%lu temp=%.2fC/%.2fF humidity=%.2f%% pressure=%.2fhPa flags=0x%02X\n",
                sourceMac[0], sourceMac[1], sourceMac[2], sourceMac[3], sourceMac[4], sourceMac[5], packet.nodeId,
                static_cast<unsigned long>(packet.sequence), packet.temperatureC, packetTemperatureF, packet.humidityRh,
                packet.pressureHpa, packet.flags);

  portENTER_CRITICAL(&nodeMux);
  NodeState &node = nodes[slot];
  node.packet = packet;
  node.receivedMs = millis();
  node.received = true;
  ++validPacketCount;
  packetPendingRedraw = true;
  const uint32_t packetTime = millis();
  recordHistory(slot, packet, packetTime);
  if (packetTime - lastAckQueuedMs[slot] >= 500 && ackQueueCount < DryBoxProtocol::NODE_COUNT) {
    PendingReadingAck &queued = pendingReadingAcks[ackQueueTail];
    queued.ack = DryBoxProtocol::ReadingAck{};
    queued.ack.magic = DryBoxProtocol::READING_ACK_MAGIC;
    queued.ack.version = DryBoxProtocol::VERSION;
    queued.ack.nodeId = packet.nodeId;
    queued.ack.status = 0;
    queued.ack.sequence = packet.sequence;
    queued.ack.checksum = DryBoxProtocol::messageChecksum(queued.ack);
    memcpy(queued.destinationMac, sourceMac, 6);
    queued.readyAtMs = packetTime + 250;
    queued.pending = true;
    ackQueueTail = (ackQueueTail + 1) % DryBoxProtocol::NODE_COUNT;
    ++ackQueueCount;
    lastAckQueuedMs[slot] = packetTime;
  }
  portEXIT_CRITICAL(&nodeMux);
}

void onSent(const uint8_t *destinationMac, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.printf("[radio] frame to %02X:%02X:%02X:%02X:%02X:%02X delivery=FAILED\n", destinationMac[0],
                  destinationMac[1], destinationMac[2], destinationMac[3], destinationMac[4], destinationMac[5]);
  }
}

void processPendingReadingAck() {
  PendingReadingAck pending;
  portENTER_CRITICAL(&nodeMux);
  if (ackQueueCount == 0) {
    portEXIT_CRITICAL(&nodeMux);
    return;
  }
  if (static_cast<int32_t>(millis() - pendingReadingAcks[ackQueueHead].readyAtMs) < 0) {
    portEXIT_CRITICAL(&nodeMux);
    return;
  }
  pending = pendingReadingAcks[ackQueueHead];
  pendingReadingAcks[ackQueueHead].pending = false;
  ackQueueHead = (ackQueueHead + 1) % DryBoxProtocol::NODE_COUNT;
  --ackQueueCount;
  portEXIT_CRITICAL(&nodeMux);
  const esp_err_t result = esp_now_send(pending.destinationMac, reinterpret_cast<const uint8_t *>(&pending.ack),
                                        sizeof(pending.ack));
  Serial.printf("[ack] node=%u sequence=%lu queue=%s\n", pending.ack.nodeId,
                static_cast<unsigned long>(pending.ack.sequence), result == ESP_OK ? "OK" : "FAILED");
}

void processPendingPair() {
  PendingPairRequest request;
  portENTER_CRITICAL(&nodeMux);
  if (!pendingPair.pending) {
    portEXIT_CRITICAL(&nodeMux);
    return;
  }
  request = pendingPair;
  pendingPair.pending = false;
  portEXIT_CRITICAL(&nodeMux);

  if (!pairingActive) return;
  if (slotPaired[selectedPairSlot] && memcmp(request.sourceMac, pairedMacs[selectedPairSlot], 6) != 0) return;
  const bool newNode = !slotPaired[selectedPairSlot];
  if (newNode) memcpy(pairedMacs[selectedPairSlot], request.sourceMac, 6);
  slotPaired[selectedPairSlot] = true;
  addPeer(request.sourceMac);
  if (newNode) savePairedSlot(selectedPairSlot);

  DryBoxProtocol::PairResponse response{};
  response.magic = DryBoxProtocol::PAIR_RESPONSE_MAGIC;
  response.version = DryBoxProtocol::VERSION;
  response.assignedNodeId = selectedPairSlot + 1;
  response.wifiChannel = request.receivedChannel;
  memcpy(response.controllerMac, localMac, 6);
  response.nonce = request.request.nonce;
  response.checksum = DryBoxProtocol::messageChecksum(response);
  // Keep pairing active until the node confirms with its first reading. Duplicate requests resend this response.
  for (uint8_t attempt = 0; attempt < 5; ++attempt) {
    const esp_err_t result = esp_now_send(request.sourceMac, reinterpret_cast<const uint8_t *>(&response), sizeof(response));
    Serial.printf("[pair] response attempt=%u slot=ACE %u queue=%s\n", attempt + 1, selectedPairSlot + 1,
                  result == ESP_OK ? "OK" : "FAILED");
    delay(50);
  }
  packetPendingRedraw = true;
  redraw();
}

void sendPairBeacon() {
  DryBoxProtocol::PairBeacon beacon{};
  beacon.magic = DryBoxProtocol::PAIR_BEACON_MAGIC;
  beacon.version = DryBoxProtocol::VERSION;
  beacon.assignedNodeId = selectedPairSlot + 1;
  beacon.reserved = pairingActive ? 1 : 0;
  wifi_second_chan_t secondaryChannel = WIFI_SECOND_CHAN_NONE;
  esp_wifi_get_channel(&beacon.wifiChannel, &secondaryChannel);
  memcpy(beacon.controllerMac, localMac, sizeof(localMac));
  beacon.checksum = DryBoxProtocol::messageChecksum(beacon);
  const esp_err_t result = esp_now_send(BROADCAST_ADDRESS, reinterpret_cast<const uint8_t *>(&beacon), sizeof(beacon));
  if (pairingActive) {
    Serial.printf("[pair] beacon slot=ACE %u channel=%u send=%d\n", selectedPairSlot + 1, beacon.wifiChannel,
                  static_cast<int>(result));
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH);

  tft.init();
  tft.setRotation(SCREEN_ROTATION);
  tft.fillScreen(COLOR_BG);
  tft.setTextWrap(false);

  touchSpi.begin(TOUCH_SCK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS_PIN);
  touch.begin(touchSpi);
  touch.setRotation(SCREEN_ROTATION);

  preferences.begin("drybox", true);
  touchMinX = preferences.getInt("touchMinX", DEFAULT_TOUCH_MIN_X);
  touchMaxX = preferences.getInt("touchMaxX", DEFAULT_TOUCH_MAX_X);
  touchMinY = preferences.getInt("touchMinY", DEFAULT_TOUCH_MIN_Y);
  touchMaxY = preferences.getInt("touchMaxY", DEFAULT_TOUCH_MAX_Y);
  touchCalibrated = preferences.getBool("touchCal", false);
  localTemperatureOffsetC = preferences.getFloat("localTempOff", 0.0f);
  preferences.end();
  if (!touchCalibrated) runTouchCalibration();

  Wire.begin(LOCAL_I2C_SDA, LOCAL_I2C_SCL);
  readLocalSensor();

  const bool wifiConnected = connectWifi();
  wifiWasConnected = wifiConnected;
  lastConnectedWifiChannel = wifiConnected ? WiFi.channel() : 0;
  if (!wifiConnected) wifiDisconnectedSinceMs = millis();

  preferences.begin("drybox", true);
  goodLimitRh = preferences.getFloat("goodRh", 30.0f);
  warningLimitRh = preferences.getFloat("warnRh", 45.0f);
  for (uint8_t i = 0; i < DryBoxProtocol::NODE_COUNT; ++i) {
    char key[8];
    snprintf(key, sizeof(key), "mac%u", i + 1);
    if (preferences.isKey(key) && preferences.getBytesLength(key) == 6) {
      preferences.getBytes(key, pairedMacs[i], 6);
      slotPaired[i] = true;
    }
  }
  preferences.end();
  if (goodLimitRh < 10 || goodLimitRh > 70) goodLimitRh = 30;
  if (warningLimitRh <= goodLimitRh || warningLimitRh > 80) warningLimitRh = 45;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  const esp_err_t protocolResult = esp_wifi_set_protocol(
      WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  WiFi.macAddress(localMac);
  Serial.printf("[radio] Display MAC %s\n", WiFi.macAddress().c_str());
  Serial.printf("[radio] maxTxPower=19.5dBm standardWiFi=%s\n", protocolResult == ESP_OK ? "enabled" : "failed");
  if (esp_now_init() != ESP_OK) {
    tft.setTextColor(TFT_RED, COLOR_BG);
    tft.drawString("ESP-NOW FAILED", 20, 100, 4);
    delay(3000);
    ESP.restart();
  }
  esp_now_register_recv_cb(onPacket);
  esp_now_register_send_cb(onSent);
  addPeer(BROADCAST_ADDRESS);
  for (uint8_t i = 0; i < DryBoxProtocol::NODE_COUNT; ++i) {
    if (slotPaired[i]) addPeer(pairedMacs[i]);
  }
  bool anyNodePaired = false;
  for (bool pairedSlot : slotPaired) anyNodePaired |= pairedSlot;
  if (!anyNodePaired) {
    pairingScreen = true;
    pairingActive = true;
    pairingEndsMs = millis() + 60000;
  }
  if (wifiConnected) startWebServer();
  if (wifiConnected) {
    setenv("TZ", weatherTimezone.c_str(), 1);
    tzset();
    configTzTime(weatherTimezone.c_str(), "pool.ntp.org", "time.nist.gov", "time.google.com");
    lastWeatherAttemptMs = millis() - WEATHER_INTERVAL_MS + 5000;
  }
  redraw();
}

void loop() {
  if (webServerStarted) webServer.handleClient();
  handleTouch();
  processPendingPair();
  processPendingReadingAck();
  const uint32_t now = millis();
  handleWifiRecovery(now);
  const uint32_t beaconInterval = pairingActive ? 500 : 1000;
  if (lastPairBeaconMs == 0 || now - lastPairBeaconMs >= beaconInterval) {
    lastPairBeaconMs = now;
    sendPairBeacon();
  }
  if (now - lastLocalSensorMs >= LOCAL_SENSOR_INTERVAL_MS) {
    readLocalSensor();
    if (weatherScreen) redraw();
  }
  if (WiFi.status() == WL_CONNECTED && weatherApiKey.length() &&
      (lastWeatherAttemptMs == 0 || now - lastWeatherAttemptMs >= WEATHER_INTERVAL_MS)) {
    lastWeatherAttemptMs = now;
    weatherClient.update(weatherApiKey, weatherLatitude, weatherLongitude, weatherImperial, weatherData, weatherError);
    if (weatherScreen) redraw();
  }
  if (pairingActive && static_cast<int32_t>(now - pairingEndsMs) >= 0) {
    pairingActive = false;
    redraw();
  }
  const uint32_t refreshInterval = detailNode >= 0 ? DETAIL_REFRESH_MS : OVERVIEW_REFRESH_MS;
  const bool packetRefresh = packetPendingRedraw && now - lastRedrawMs >= PACKET_REDRAW_DELAY_MS;
  if (!settingsScreen && (packetRefresh || now - lastRedrawMs >= refreshInterval)) {
    packetPendingRedraw = false;
    lastRedrawMs = now;
    redraw();
  }
  delay(10);
}
