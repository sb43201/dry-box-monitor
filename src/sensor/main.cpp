#include <Arduino.h>
#include <Adafruit_AHTX0.h>
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
constexpr uint8_t BROADCAST_ADDRESS[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

Adafruit_AHTX0 aht;
Preferences preferences;
bool sensorReady = false;
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
  Serial.printf("[sensor] AHT20=%s on SDA=%u SCL=%u\n", sensorReady ? "ready" : "missing", SDA_PIN, SCL_PIN);
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
      packet.temperatureC = temperature.temperature + TEMPERATURE_OFFSET_C;
      packet.humidityRh = constrain(humidity.relative_humidity + HUMIDITY_OFFSET_RH, 0.0f, 100.0f);
      packet.flags |= DryBoxProtocol::FLAG_SENSOR_OK;
    } else {
      sensorReady = false;
    }
  }

  packet.checksum = DryBoxProtocol::packetChecksum(packet);
  const esp_err_t result = esp_now_send(controllerMac, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  if (result == ESP_OK) flashTransmitLed();
  Serial.printf("[send] node=%u seq=%lu temp=%.1fC rh=%.1f%% pressure=%.1fhPa result=%s\n", packet.nodeId,
                static_cast<unsigned long>(packet.sequence), packet.temperatureC, packet.humidityRh,
                packet.pressureHpa, result == ESP_OK ? "queued" : "failed");
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
    Serial.printf("[pair] Paired as ACE %u\n", nodeId);
    return;
  }

  if (length == sizeof(DryBoxProtocol::ReadingAck)) {
    DryBoxProtocol::ReadingAck ack;
    memcpy(&ack, data, sizeof(ack));
    if (paired && sameMac(sourceMac, controllerMac) && DryBoxProtocol::valid(ack) && ack.nodeId == nodeId) {
      Serial.printf("[ack] controller accepted node=%u sequence=%lu status=%u\n", ack.nodeId,
                    static_cast<unsigned long>(ack.sequence), ack.status);
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
  loadPairing();
  pairingNonce = esp_random();
  if (!startEspNow()) {
    Serial.println("[radio] ESP-NOW initialization failed");
    delay(2000);
    ESP.restart();
  }
  Serial.println(paired ? "[pair] Saved controller loaded" : "[pair] Unpaired; advertising");
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
