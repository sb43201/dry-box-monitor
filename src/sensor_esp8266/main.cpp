#include <Arduino.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
extern "C" {
#include <espnow.h>
#include <user_interface.h>
}

#include "protocol.h"

namespace {
constexpr uint8_t SDA_PIN = D2;  // GPIO4
constexpr uint8_t SCL_PIN = D1;  // GPIO5
constexpr uint8_t LED_PIN = LED_BUILTIN;
constexpr uint32_t SEND_INTERVAL_MS = 30000;
constexpr uint32_t RETRY_INTERVAL_MS = 5000;
constexpr uint32_t PAIR_INTERVAL_MS = 2000;
constexpr uint32_t SENSOR_RETRY_MS = 5000;
constexpr uint32_t BEACON_TIMEOUT_MS = 5000;
constexpr uint32_t SCAN_DWELL_MS = 1200;
constexpr uint32_t LED_FLASH_MS = 120;
constexpr uint8_t OLED_ADDRESS = 0x3C;
constexpr uint8_t BROADCAST_MAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
constexpr uint32_t SETTINGS_MAGIC = 0x45383236;  // E826

struct __attribute__((packed)) SavedPairing {
  uint32_t magic;
  uint8_t paired;
  uint8_t nodeId;
  uint8_t channel;
  uint8_t controllerMac[6];
  uint32_t checksum;
};

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
Adafruit_SSD1306 oled(128, 32, &Wire, -1);
bool sensorReady = false;
bool bmpReady = false;
bool oledReady = false;
bool paired = false;
uint8_t nodeId = 0;
uint8_t pairedChannel = 1;
uint8_t controllerMac[6]{};
uint8_t pairingChannel = 1;
uint32_t pairingNonce = 0;
uint32_t sequenceNumber = 0;
uint32_t lastSendMs = 0;
uint32_t lastAttemptMs = 0;
uint32_t lastPairMs = 0;
uint32_t lastSensorAttemptMs = 0;
uint32_t lastBeaconMs = 0;
uint32_t lastScanMs = 0;
uint8_t scanChannel = 1;
bool scanning = false;
float lastTemperatureC = NAN;
float lastHumidityRh = NAN;
DryBoxProtocol::SensorPacket pendingReading{};
bool awaitingAck = false;
uint32_t ledOffAtMs = 0;

volatile bool beaconPending = false;
volatile uint8_t beaconChannel = 0;
volatile bool controllerDiscoveryPending = false;
uint8_t discoveredControllerMac[6]{};
volatile uint8_t discoveredControllerChannel = 1;
volatile bool pairResponsePending = false;
DryBoxProtocol::PairResponse pendingPairResponse{};
volatile bool ackPending = false;
DryBoxProtocol::ReadingAck pendingAck{};
volatile bool unpairPending = false;

bool sameMac(const uint8_t *a, const uint8_t *b) { return memcmp(a, b, 6) == 0; }

uint32_t savedChecksum(const SavedPairing &saved) {
  return DryBoxProtocol::checksum(reinterpret_cast<const uint8_t *>(&saved), offsetof(SavedPairing, checksum));
}

void savePairing() {
  SavedPairing saved{};
  saved.magic = SETTINGS_MAGIC;
  saved.paired = paired ? 1 : 0;
  saved.nodeId = nodeId;
  saved.channel = pairedChannel;
  memcpy(saved.controllerMac, controllerMac, 6);
  saved.checksum = savedChecksum(saved);
  EEPROM.put(0, saved);
  EEPROM.commit();
}

void loadPairing() {
  SavedPairing saved{};
  EEPROM.get(0, saved);
  if (saved.magic != SETTINGS_MAGIC || saved.checksum != savedChecksum(saved) || !saved.paired ||
      saved.nodeId < 1 || saved.nodeId > DryBoxProtocol::NODE_COUNT || saved.channel < 1 || saved.channel > 13) return;
  paired = true;
  nodeId = saved.nodeId;
  pairedChannel = saved.channel;
  memcpy(controllerMac, saved.controllerMac, 6);
}

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
    oled.printf("%.1f C / %.1f F", lastTemperatureC, lastTemperatureC * 1.8f + 32.0f);
    oled.setCursor(0, 16);
    oled.printf("Humidity %.1f%%", lastHumidityRh);
  } else {
    oled.print(sensorReady ? "Waiting for reading" : "AHT20 SENSOR ERROR");
  }
  oled.setCursor(0, 24);
  oled.print(status);
  oled.display();
}

void addPeer(const uint8_t *mac, uint8_t channel) {
  uint8_t mutableMac[6];
  memcpy(mutableMac, mac, 6);
  if (esp_now_is_peer_exist(mutableMac)) esp_now_del_peer(mutableMac);
  const int result = esp_now_add_peer(mutableMac, ESP_NOW_ROLE_COMBO, channel, nullptr, 0);
  Serial.printf("[radio] add peer %02X:%02X:%02X:%02X:%02X:%02X channel=%u result=%d\n", mac[0], mac[1],
                mac[2], mac[3], mac[4], mac[5], channel, result);
}

void clearPairing() {
  paired = false;
  nodeId = 0;
  pairedChannel = 1;
  memset(controllerMac, 0, 6);
  pairingChannel = 1;
  pairingNonce = os_random();
  scanning = false;
  awaitingAck = false;
  savePairing();
  Serial.println("[pair] Pairing cleared; advertising for a controller");
  drawOled("PAIRING");
}

bool startSensors() {
  lastSensorAttemptMs = millis();
  sensorReady = aht.begin(&Wire);
  bmpReady = bmp.begin(0x76, 0x58) || bmp.begin(0x77, 0x58);
  Serial.printf("[sensor-status] AHT20=%s BMP280=%s SDA=%u SCL=%u\n", sensorReady ? "ready" : "missing",
                bmpReady ? "ready" : "not installed", SDA_PIN, SCL_PIN);
  return sensorReady;
}

void IRAM_ATTR onSent(uint8_t *mac, uint8_t status) {
  Serial.printf("[radio] frame to %02X:%02X:%02X:%02X:%02X:%02X delivery=%s\n", mac[0], mac[1], mac[2],
                mac[3], mac[4], mac[5], status == 0 ? "SUCCESS" : "FAILED");
}

void IRAM_ATTR onReceive(uint8_t *mac, uint8_t *data, uint8_t length) {
  if (length == sizeof(DryBoxProtocol::PairBeacon)) {
    DryBoxProtocol::PairBeacon beacon{};
    memcpy(&beacon, data, sizeof(beacon));
    if (!DryBoxProtocol::valid(beacon)) return;
    if (paired && sameMac(mac, controllerMac)) {
      beaconChannel = beacon.wifiChannel;
      beaconPending = true;
    } else if (!paired && beacon.reserved == 1 && sameMac(mac, beacon.controllerMac)) {
      memcpy(discoveredControllerMac, mac, 6);
      discoveredControllerChannel = beacon.wifiChannel;
      controllerDiscoveryPending = true;
    }
    return;
  }
  if (length == sizeof(DryBoxProtocol::PairResponse)) {
    DryBoxProtocol::PairResponse response{};
    memcpy(&response, data, sizeof(response));
    if (!paired && DryBoxProtocol::valid(response) && response.nonce == pairingNonce &&
        sameMac(mac, response.controllerMac)) {
      pendingPairResponse = response;
      pairResponsePending = true;
    }
    return;
  }
  if (length == sizeof(DryBoxProtocol::ReadingAck)) {
    DryBoxProtocol::ReadingAck ack{};
    memcpy(&ack, data, sizeof(ack));
    if (paired && sameMac(mac, controllerMac) && DryBoxProtocol::valid(ack) && ack.nodeId == nodeId) {
      pendingAck = ack;
      ackPending = true;
    }
    return;
  }
  if (length == sizeof(DryBoxProtocol::UnpairCommand)) {
    DryBoxProtocol::UnpairCommand command{};
    memcpy(&command, data, sizeof(command));
    if (paired && sameMac(mac, controllerMac) && DryBoxProtocol::valid(command) && command.nodeId == nodeId) {
      unpairPending = true;
    }
  }
}

void sendPairRequest() {
  wifi_set_channel(pairingChannel);
  DryBoxProtocol::PairRequest request{};
  request.magic = DryBoxProtocol::PAIR_REQUEST_MAGIC;
  request.version = DryBoxProtocol::VERSION;
  request.nonce = pairingNonce;
  request.checksum = DryBoxProtocol::messageChecksum(request);
  for (uint8_t attempt = 0; attempt < 2; ++attempt) {
    const uint8_t *destination = controllerMac[0] || controllerMac[1] ? controllerMac : BROADCAST_MAC;
    const int result = esp_now_send(const_cast<uint8_t *>(destination), reinterpret_cast<uint8_t *>(&request),
                                    sizeof(request));
    Serial.printf("[pair] request channel=%u attempt=%u send=%d nonce=%lu\n", pairingChannel, attempt + 1, result,
                  static_cast<unsigned long>(pairingNonce));
    delay(50);
  }
  pairingChannel = pairingChannel >= 13 ? 1 : pairingChannel + 1;
}

void transmitReading(bool retry) {
  wifi_set_channel(pairedChannel);
  digitalWrite(LED_PIN, LOW);
  ledOffAtMs = millis() + LED_FLASH_MS;
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    esp_now_send(controllerMac, reinterpret_cast<uint8_t *>(&pendingReading), sizeof(pendingReading));
    delay(50);
  }
  for (uint8_t attempt = 0; attempt < 2; ++attempt) {
    esp_now_send(const_cast<uint8_t *>(BROADCAST_MAC), reinterpret_cast<uint8_t *>(&pendingReading),
                 sizeof(pendingReading));
    delay(40);
  }
  lastAttemptMs = millis();
  Serial.printf("[send] node=%u seq=%lu retry=%s\n", nodeId, static_cast<unsigned long>(pendingReading.sequence),
                retry ? "yes" : "no");
  drawOled("TX QUEUED");
}

void sendReading() {
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
    sensors_event_t humidity, temperature;
    aht.getEvent(&humidity, &temperature);
    if (isfinite(temperature.temperature) && isfinite(humidity.relative_humidity)) {
      packet.temperatureC = temperature.temperature;
      packet.humidityRh = humidity.relative_humidity;
      packet.flags |= DryBoxProtocol::FLAG_SENSOR_OK;
      lastTemperatureC = packet.temperatureC;
      lastHumidityRh = packet.humidityRh;
      Serial.printf("[AHT20] status=OK rawTemp=%.2fC/%.2fF rawHumidity=%.2f%%\n", packet.temperatureC,
                    packet.temperatureC * 1.8f + 32.0f, packet.humidityRh);
    } else {
      sensorReady = false;
      Serial.println("[AHT20] status=READ_ERROR");
    }
  } else {
    Serial.println("[AHT20] status=MISSING no raw data");
  }
  if (bmpReady) {
    const float temperature = bmp.readTemperature();
    const float pressure = bmp.readPressure() / 100.0f;
    if (isfinite(temperature) && isfinite(pressure)) {
      packet.pressureHpa = pressure;
      packet.flags |= DryBoxProtocol::FLAG_PRESSURE_OK;
      Serial.printf("[BMP280] status=OK rawTemp=%.2fC/%.2fF rawPressure=%.2fhPa\n", temperature,
                    temperature * 1.8f + 32.0f, pressure);
    }
  } else {
    Serial.println("[BMP280] status=NOT_INSTALLED pressure omitted");
  }
  packet.checksum = DryBoxProtocol::packetChecksum(packet);
  pendingReading = packet;
  awaitingAck = true;
  transmitReading(false);
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Wire.begin(SDA_PIN, SCL_PIN);
  startSensors();
  Wire.beginTransmission(OLED_ADDRESS);
  if (Wire.endTransmission() == 0) oledReady = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  EEPROM.begin(64);
  loadPairing();
  pairingNonce = os_random();
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setOutputPower(20.5f);
  if (esp_now_init() != 0) {
    Serial.println("[radio] ESP-NOW initialization failed");
    ESP.restart();
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceive);
  addPeer(BROADCAST_MAC, 0);
  if (paired) {
    wifi_set_channel(pairedChannel);
    addPeer(controllerMac, pairedChannel);
  }
  lastBeaconMs = millis();
  Serial.printf("[radio] ESP8266 MAC %s maxTxPower=20.5dBm\n", WiFi.macAddress().c_str());
  Serial.println(paired ? "[pair] Saved controller loaded" : "[pair] Unpaired; advertising");
  drawOled(paired ? "READY" : "PAIRING");
}

void loop() {
  const uint32_t now = millis();
  if (ledOffAtMs && static_cast<int32_t>(now - ledOffAtMs) >= 0) {
    digitalWrite(LED_PIN, HIGH);
    ledOffAtMs = 0;
  }
  if (controllerDiscoveryPending) {
    controllerDiscoveryPending = false;
    memcpy(controllerMac, discoveredControllerMac, 6);
    pairingChannel = discoveredControllerChannel;
    wifi_set_channel(pairingChannel);
    addPeer(controllerMac, pairingChannel);
    lastPairMs = 0;
    Serial.printf("[pair] controller beacon received channel=%u\n", pairingChannel);
  }
  if (unpairPending) {
    unpairPending = false;
    clearPairing();
  }
  if (pairResponsePending) {
    pairResponsePending = false;
    paired = true;
    nodeId = pendingPairResponse.assignedNodeId;
    pairedChannel = pendingPairResponse.wifiChannel;
    memcpy(controllerMac, pendingPairResponse.controllerMac, 6);
    wifi_set_channel(pairedChannel);
    addPeer(controllerMac, pairedChannel);
    savePairing();
    lastBeaconMs = now;
    lastSendMs = 0;
    Serial.printf("[pair] Paired as ACE %u on channel %u\n", nodeId, pairedChannel);
  }
  if (ackPending) {
    ackPending = false;
    Serial.printf("[ack] controller accepted node=%u sequence=%lu status=%u\n", pendingAck.nodeId,
                  static_cast<unsigned long>(pendingAck.sequence), pendingAck.status);
    if (awaitingAck && pendingAck.sequence == pendingReading.sequence) awaitingAck = false;
    drawOled("ACK / READY");
  }
  if (beaconPending) {
    beaconPending = false;
    if (beaconChannel >= 1 && beaconChannel <= 13) {
      const bool recovered = scanning || beaconChannel != pairedChannel;
      const bool changed = beaconChannel != pairedChannel;
      pairedChannel = beaconChannel;
      lastBeaconMs = now;
      scanning = false;
      if (recovered) {
        wifi_set_channel(pairedChannel);
        addPeer(controllerMac, pairedChannel);
        if (changed) savePairing();
        Serial.printf("[radio] controller beacon recovered channel=%u saved=%s\n", pairedChannel,
                      changed ? "yes" : "no");
      }
    }
  }
  if (!sensorReady && now - lastSensorAttemptMs >= SENSOR_RETRY_MS) startSensors();
  if (paired && !scanning && now - lastBeaconMs >= BEACON_TIMEOUT_MS) {
    scanning = true;
    scanChannel = pairedChannel >= 13 ? 1 : pairedChannel + 1;
    lastScanMs = 0;
    Serial.printf("[radio] controller beacon lost; scanning from channel %u\n", scanChannel);
    drawOled("FIND CHANNEL");
  }
  if (paired && scanning && (lastScanMs == 0 || now - lastScanMs >= SCAN_DWELL_MS)) {
    wifi_set_channel(scanChannel);
    Serial.printf("[radio] scanning channel=%u\n", scanChannel);
    lastScanMs = now;
    scanChannel = scanChannel >= 13 ? 1 : scanChannel + 1;
  }
  if (!paired && (lastPairMs == 0 || now - lastPairMs >= PAIR_INTERVAL_MS)) {
    lastPairMs = now;
    sendPairRequest();
  } else if (paired && !scanning && (lastSendMs == 0 || now - lastSendMs >= SEND_INTERVAL_MS)) {
    lastSendMs = now;
    sendReading();
  }
  if (paired && !scanning && awaitingAck && now - lastAttemptMs >= RETRY_INTERVAL_MS) transmitReading(true);
  delay(20);
}
