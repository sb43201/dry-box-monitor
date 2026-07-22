#include <Arduino.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "protocol.h"

namespace {
constexpr uint8_t SDA_PIN = 4;
constexpr uint8_t SCL_PIN = 5;
constexpr uint8_t PAIR_BUTTON_PIN = 9;  // BOOT button, active low
constexpr uint8_t ONBOARD_LED_PIN = 8;  // ESP32-C3 Super Mini blue LED, active low
constexpr uint8_t LED_ON = LOW;
constexpr uint8_t LED_OFF = HIGH;
constexpr uint32_t LED_FLASH_MS = 120;
constexpr uint32_t SEND_INTERVAL_MS = 30000;
constexpr uint32_t PAIR_REQUEST_INTERVAL_MS = 2000;
constexpr uint32_t SENSOR_RETRY_MS = 5000;
constexpr uint32_t UNPAIR_HOLD_MS = 5000;
constexpr float TEMPERATURE_OFFSET_C = 0.0f;
constexpr float HUMIDITY_OFFSET_RH = 0.0f;
constexpr uint8_t OLED_ADDRESS = 0x3C;
constexpr int16_t OLED_WIDTH = 128;
constexpr int16_t OLED_HEIGHT = 32;
constexpr uint8_t BROADCAST_ADDRESS[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Preferences preferences;
bool sensorReady = false;
bool bmpReady = false;
bool oledReady = false;
bool paired = false;
uint8_t nodeId = 0;
uint8_t controllerMac[6]{};
uint8_t pairedChannel = 1;
uint8_t pairingChannel = 1;
uint32_t pairingNonce = 0;
uint32_t sequenceNumber = 0;
uint32_t lastSendMs = 0;
uint32_t lastPairRequestMs = 0;
uint32_t lastSensorAttemptMs = 0;
uint32_t buttonDownMs = 0;
bool buttonHandled = false;
volatile bool remoteUnpairRequested = false;
uint32_t ledOffAtMs = 0;
float lastTemperatureC = NAN;
float lastHumidityRh = NAN;
uint32_t lastDisplayedSequence = 0;
volatile bool oledDirty = false;

void drawOled(const char *status) {
  if (!oledReady) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  if (paired) oled.printf("ACE %u  CH %u", nodeId, pairedChannel);
  else oled.printf("PAIRING  CH %u", pairingChannel);
  oled.setCursor(0, 8);
  if (isfinite(lastTemperatureC)) {
    const float temperatureF = lastTemperatureC * 9.0f / 5.0f + 32.0f;
    oled.printf("%.1f C / %.1f F", lastTemperatureC, temperatureF);
    oled.setCursor(0, 16);
    oled.printf("Humidity %.1f%%", lastHumidityRh);
  } else {
    oled.print(sensorReady ? "Waiting for reading" : "AHT20 SENSOR ERROR");
    oled.setCursor(0, 16);
    oled.print(paired ? "Check sensor wiring" : "Seeking controller");
  }
  oled.setCursor(0, 24);
  oled.printf("%s", status);
  if (lastDisplayedSequence) oled.printf("  #%lu", static_cast<unsigned long>(lastDisplayedSequence));
  oled.display();
}

void startOled() {
  Wire.beginTransmission(OLED_ADDRESS);
  if (Wire.endTransmission() != 0) {
    oledReady = false;
    Serial.printf("[display] SSD1306 128x32 at 0x%02X=not detected\n", OLED_ADDRESS);
    return;
  }
  oledReady = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS, false, false);
  Serial.printf("[display] SSD1306 128x32 at 0x%02X=%s\n", OLED_ADDRESS, oledReady ? "ready" : "not detected");
  drawOled("STARTING");
}

void flashTransmitLed() {
  digitalWrite(ONBOARD_LED_PIN, LED_ON);
  ledOffAtMs = millis() + LED_FLASH_MS;
}

void onSent(const uint8_t *destinationMac, esp_now_send_status_t status) {
  Serial.printf("[radio] frame to %02X:%02X:%02X:%02X:%02X:%02X delivery=%s\n", destinationMac[0],
                destinationMac[1], destinationMac[2], destinationMac[3], destinationMac[4], destinationMac[5],
                status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAILED");
}

bool sameMac(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

void addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void savePairing() {
  preferences.begin("drybox", false);
  preferences.putBool("paired", paired);
  preferences.putUChar("nodeId", nodeId);
  if (paired) preferences.putBytes("ctrlMac", controllerMac, sizeof(controllerMac));
  if (paired) preferences.putUChar("channel", pairedChannel);
  else preferences.remove("ctrlMac");
  preferences.end();
}

void clearPairing() {
  if (paired && esp_now_is_peer_exist(controllerMac)) esp_now_del_peer(controllerMac);
  paired = false;
  nodeId = 0;
  memset(controllerMac, 0, sizeof(controllerMac));
  pairedChannel = 1;
  pairingChannel = 1;
  pairingNonce = esp_random();
  savePairing();
  lastPairRequestMs = 0;
  Serial.println("[pair] Pairing cleared; advertising for a controller");
  drawOled("PAIRING");
}

void loadPairing() {
  preferences.begin("drybox", true);
  paired = preferences.getBool("paired", false);
  nodeId = preferences.getUChar("nodeId", 0);
  const size_t macLength = preferences.getBytesLength("ctrlMac");
  if (paired && macLength == sizeof(controllerMac)) preferences.getBytes("ctrlMac", controllerMac, sizeof(controllerMac));
  else paired = false;
  preferences.end();
  if (nodeId < 1 || nodeId > DryBoxProtocol::NODE_COUNT) paired = false;
  if (paired) {
    preferences.begin("drybox", true);
    pairedChannel = preferences.getUChar("channel", 1);
    preferences.end();
    if (pairedChannel < 1 || pairedChannel > 13) paired = false;
  }
}

bool startSensor() {
  lastSensorAttemptMs = millis();
  sensorReady = aht.begin(&Wire);
  bmpReady = bmp.begin(0x76, 0x58) || bmp.begin(0x77, 0x58);
  Serial.printf("[sensor-status] AHT20=%s BMP280=%s SDA=%u SCL=%u\n", sensorReady ? "ready" : "missing",
                bmpReady ? "ready" : "not installed", SDA_PIN, SCL_PIN);
  return sensorReady;
}

void sendPairRequest() {
  esp_wifi_set_channel(pairingChannel, WIFI_SECOND_CHAN_NONE);
  DryBoxProtocol::PairRequest request{};
  request.magic = DryBoxProtocol::PAIR_REQUEST_MAGIC;
  request.version = DryBoxProtocol::VERSION;
  request.nonce = pairingNonce;
  request.checksum = DryBoxProtocol::messageChecksum(request);
  esp_now_send(BROADCAST_ADDRESS, reinterpret_cast<const uint8_t *>(&request), sizeof(request));
  Serial.printf("[pair] Pairing request on Wi-Fi channel %u\n", pairingChannel);
  pairingChannel = pairingChannel >= 13 ? 1 : pairingChannel + 1;
  drawOled("PAIR REQUEST");
}

void sendReading() {
  if (!paired) return;
  DryBoxProtocol::SensorPacket packet{};
  packet.magic = DryBoxProtocol::MAGIC;
  packet.version = DryBoxProtocol::VERSION;
  packet.nodeId = nodeId;
  packet.sequence = ++sequenceNumber;
  packet.uptimeSeconds = millis() / 1000UL;
  packet.temperatureC = NAN;
  packet.humidityRh = NAN;
  packet.pressureHpa = NAN;

  if (sensorReady) {
    sensors_event_t humidity;
    sensors_event_t temperature;
    aht.getEvent(&humidity, &temperature);
    if (isfinite(temperature.temperature) && isfinite(humidity.relative_humidity)) {
      const float rawTemperatureF = temperature.temperature * 9.0f / 5.0f + 32.0f;
      packet.temperatureC = temperature.temperature + TEMPERATURE_OFFSET_C;
      packet.humidityRh = constrain(humidity.relative_humidity + HUMIDITY_OFFSET_RH, 0.0f, 100.0f);
      packet.flags |= DryBoxProtocol::FLAG_SENSOR_OK;
      lastTemperatureC = packet.temperatureC;
      lastHumidityRh = packet.humidityRh;
      const float correctedTemperatureF = packet.temperatureC * 9.0f / 5.0f + 32.0f;
      Serial.printf("[AHT20] status=OK rawTemp=%.2fC/%.2fF rawHumidity=%.2f%% correctedTemp=%.2fC/%.2fF correctedHumidity=%.2f%%\n",
                    temperature.temperature, rawTemperatureF, humidity.relative_humidity, packet.temperatureC,
                    correctedTemperatureF, packet.humidityRh);
    } else {
      sensorReady = false;
      Serial.println("[AHT20] status=READ_ERROR raw data invalid");
    }
  } else {
    Serial.println("[AHT20] status=MISSING no raw data");
  }

  if (bmpReady) {
    const float bmpTemperatureC = bmp.readTemperature();
    const float bmpPressureHpa = bmp.readPressure() / 100.0f;
    if (isfinite(bmpTemperatureC) && isfinite(bmpPressureHpa) && bmpPressureHpa > 300.0f && bmpPressureHpa < 1200.0f) {
      const float bmpTemperatureF = bmpTemperatureC * 9.0f / 5.0f + 32.0f;
      packet.pressureHpa = bmpPressureHpa;
      packet.flags |= DryBoxProtocol::FLAG_PRESSURE_OK;
      Serial.printf("[BMP280] status=OK rawTemp=%.2fC/%.2fF rawPressure=%.2fhPa\n", bmpTemperatureC,
                    bmpTemperatureF, bmpPressureHpa);
    } else {
      Serial.println("[BMP280] status=READ_ERROR raw data invalid");
    }
  } else {
    Serial.println("[BMP280] status=NOT_INSTALLED pressure omitted");
  }

  packet.checksum = DryBoxProtocol::packetChecksum(packet);
  const esp_err_t result = esp_now_send(controllerMac, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  if (result == ESP_OK) flashTransmitLed();
  lastDisplayedSequence = packet.sequence;
  drawOled(result == ESP_OK ? "TX QUEUED" : "TX FAILED");
  Serial.printf("[send] node=%u seq=%lu sensor=%s pressure=%s temp=%.2fC/%.2fF humidity=%.2f%% pressureHpa=%.2f result=%s\n",
                packet.nodeId, static_cast<unsigned long>(packet.sequence),
                packet.flags & DryBoxProtocol::FLAG_SENSOR_OK ? "OK" : "ERROR",
                packet.flags & DryBoxProtocol::FLAG_PRESSURE_OK ? "OK" : "N/A", packet.temperatureC,
                packet.temperatureC * 9.0f / 5.0f + 32.0f, packet.humidityRh, packet.pressureHpa,
                result == ESP_OK ? "queued" : "failed");
}

void onPacket(const uint8_t *sourceMac, const uint8_t *data, int length) {
  if (length == sizeof(DryBoxProtocol::PairResponse)) {
    DryBoxProtocol::PairResponse response;
    memcpy(&response, data, sizeof(response));
    if (paired || !DryBoxProtocol::valid(response) || response.nonce != pairingNonce ||
        !sameMac(sourceMac, response.controllerMac)) return;
    memcpy(controllerMac, sourceMac, sizeof(controllerMac));
    nodeId = response.assignedNodeId;
    pairedChannel = response.wifiChannel;
    paired = true;
    addPeer(controllerMac);
    savePairing();
    lastSendMs = 0;
    oledDirty = true;
    Serial.printf("[pair] Paired as ACE %u\n", nodeId);
    return;
  }

  if (length == sizeof(DryBoxProtocol::ReadingAck)) {
    DryBoxProtocol::ReadingAck ack;
    memcpy(&ack, data, sizeof(ack));
    if (paired && sameMac(sourceMac, controllerMac) && DryBoxProtocol::valid(ack) && ack.nodeId == nodeId) {
      Serial.printf("[ack] controller accepted node=%u sequence=%lu status=%u\n", ack.nodeId,
                    static_cast<unsigned long>(ack.sequence), ack.status);
      oledDirty = true;
    }
    return;
  }

  if (length == sizeof(DryBoxProtocol::UnpairCommand)) {
    DryBoxProtocol::UnpairCommand command;
    memcpy(&command, data, sizeof(command));
    if (paired && sameMac(sourceMac, controllerMac) && DryBoxProtocol::valid(command) && command.nodeId == nodeId) {
      remoteUnpairRequested = true;
    }
  }
}

bool startEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.printf("[radio] MAC %s\n", WiFi.macAddress().c_str());
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onPacket);
  esp_now_register_send_cb(onSent);
  addPeer(BROADCAST_ADDRESS);
  if (paired) {
    esp_wifi_set_channel(pairedChannel, WIFI_SECOND_CHAN_NONE);
    addPeer(controllerMac);
  }
  return true;
}

void handlePairButton() {
  const bool pressed = digitalRead(PAIR_BUTTON_PIN) == LOW;
  if (!pressed) {
    buttonDownMs = 0;
    buttonHandled = false;
    return;
  }
  if (buttonDownMs == 0) buttonDownMs = millis();
  if (!buttonHandled && millis() - buttonDownMs >= UNPAIR_HOLD_MS) {
    buttonHandled = true;
    clearPairing();
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PAIR_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, LED_OFF);
  Wire.begin(SDA_PIN, SCL_PIN);
  startSensor();
  startOled();
  loadPairing();
  pairingNonce = esp_random();
  if (!startEspNow()) {
    Serial.println("[radio] ESP-NOW initialization failed");
    delay(2000);
    ESP.restart();
  }
  Serial.println(paired ? "[pair] Saved controller loaded" : "[pair] Unpaired; advertising");
  drawOled(paired ? "READY" : "PAIRING");
}

void loop() {
  const uint32_t now = millis();
  if (ledOffAtMs && static_cast<int32_t>(now - ledOffAtMs) >= 0) {
    digitalWrite(ONBOARD_LED_PIN, LED_OFF);
    ledOffAtMs = 0;
  }
  if (remoteUnpairRequested) {
    remoteUnpairRequested = false;
    clearPairing();
  }
  handlePairButton();
  if (oledDirty) {
    oledDirty = false;
    drawOled(paired ? "ACK / READY" : "PAIRING");
  }
  if (!sensorReady && now - lastSensorAttemptMs >= SENSOR_RETRY_MS) startSensor();

  if (!paired && (lastPairRequestMs == 0 || now - lastPairRequestMs >= PAIR_REQUEST_INTERVAL_MS)) {
    lastPairRequestMs = now;
    sendPairRequest();
  } else if (paired && (lastSendMs == 0 || now - lastSendMs >= SEND_INTERVAL_MS)) {
    lastSendMs = now;
    sendReading();
  }
  delay(20);
}
