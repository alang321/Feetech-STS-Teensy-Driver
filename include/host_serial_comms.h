#pragma once

#include "arduino.h"
#include <config.h>

struct inbound_packet {
  const uint8_t startByte1 = 0xBF;
  const uint8_t startByte2 = 0xFF;
  uint8_t cmd_id;
  uint8_t data[255];
  uint8_t checksum;
} inbound_pkt;

struct outbound_packet {
  const uint8_t startByte1 = 0xBF;
  const uint8_t startByte2 = 0xFF;
  uint8_t reply_id;
  uint8_t data[255];
  uint8_t checksum;
} outbound_pkt;

byte last_received_byte = 0;
bool last_byte_valid = false;

HardwareSerial serial_host_comms = SERIAL_COMMS;

void initSerialComms();
void sendData(uint8_t* data, size_t length);
bool receiveValidPacket();

uint8_t calculateChecksum(uint8_t* data, size_t length);