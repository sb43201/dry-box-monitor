#pragma once

#include <Arduino.h>
#include <stddef.h>

namespace DryBoxProtocol {

constexpr uint32_t MAGIC = 0x44525942;  // "DRYB"
constexpr uint8_t VERSION = 1;
constexpr uint8_t NODE_COUNT = 10;
constexpr uint8_t FLAG_SENSOR_OK = 0x01;
constexpr uint8_t FLAG_PRESSURE_OK = 0x02;
constexpr uint32_t PAIR_REQUEST_MAGIC = 0x50414952;   // "PAIR"
constexpr uint32_t PAIR_RESPONSE_MAGIC = 0x50414944;  // "PAID"
constexpr uint32_t UNPAIR_MAGIC = 0x554E5041;         // "UNPA"
constexpr uint32_t READING_ACK_MAGIC = 0x41434B52;    // "ACKR"

struct __attribute__((packed)) SensorPacket {
  uint32_t magic;
  uint8_t version;
  uint8_t nodeId;
  uint8_t flags;
  uint8_t reserved;
  uint32_t sequence;
  uint32_t uptimeSeconds;
  float temperatureC;
  float humidityRh;
  float pressureHpa;
  uint32_t checksum;
};

struct __attribute__((packed)) PairRequest {
  uint32_t magic;
  uint8_t version;
  uint8_t reserved[3];
  uint32_t nonce;
  uint32_t checksum;
};

struct __attribute__((packed)) PairResponse {
  uint32_t magic;
  uint8_t version;
  uint8_t assignedNodeId;
  uint8_t wifiChannel;
  uint8_t reserved;
  uint8_t controllerMac[6];
  uint32_t nonce;
  uint32_t checksum;
};

struct __attribute__((packed)) UnpairCommand {
  uint32_t magic;
  uint8_t version;
  uint8_t nodeId;
  uint8_t reserved[2];
  uint32_t checksum;
};

struct __attribute__((packed)) ReadingAck {
  uint32_t magic;
  uint8_t version;
  uint8_t nodeId;
  uint8_t status;
  uint8_t reserved;
  uint32_t sequence;
  uint32_t checksum;
};

inline uint32_t checksum(const uint8_t *data, size_t length) {
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < length; ++i) {
    hash ^= data[i];
    hash *= 16777619UL;
  }
  return hash;
}

inline uint32_t packetChecksum(const SensorPacket &packet) {
  return checksum(reinterpret_cast<const uint8_t *>(&packet), offsetof(SensorPacket, checksum));
}

inline bool valid(const SensorPacket &packet) {
  return packet.magic == MAGIC && packet.version == VERSION && packet.nodeId >= 1 &&
         packet.nodeId <= NODE_COUNT && packet.checksum == packetChecksum(packet);
}

template <typename T>
inline uint32_t messageChecksum(const T &message) {
  return checksum(reinterpret_cast<const uint8_t *>(&message), offsetof(T, checksum));
}

inline bool valid(const PairRequest &message) {
  return message.magic == PAIR_REQUEST_MAGIC && message.version == VERSION &&
         message.checksum == messageChecksum(message);
}

inline bool valid(const PairResponse &message) {
  return message.magic == PAIR_RESPONSE_MAGIC && message.version == VERSION &&
         message.assignedNodeId >= 1 && message.assignedNodeId <= NODE_COUNT &&
         message.wifiChannel >= 1 && message.wifiChannel <= 13 &&
         message.checksum == messageChecksum(message);
}

inline bool valid(const UnpairCommand &message) {
  return message.magic == UNPAIR_MAGIC && message.version == VERSION &&
         message.nodeId >= 1 && message.nodeId <= NODE_COUNT &&
         message.checksum == messageChecksum(message);
}

inline bool valid(const ReadingAck &message) {
  return message.magic == READING_ACK_MAGIC && message.version == VERSION &&
         message.nodeId >= 1 && message.nodeId <= NODE_COUNT &&
         message.checksum == messageChecksum(message);
}

}  // namespace DryBoxProtocol
