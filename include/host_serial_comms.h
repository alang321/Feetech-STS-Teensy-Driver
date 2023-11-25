#pragma once

#include "arduino.h"
#include <config.h>

class outbound_packet {
private:
    const uint8_t startByte1 = 0xBF;
    const uint8_t startByte2 = 0xFF;
    void* data; //here the data includes a reply_id
    uint8_t data_length;
    uint8_t checksum;

    uint8_t calculateChecksum();
public:
    outbound_packet(void* data, uint8_t data_length);
    uint8_t* get_buffer();
    uint8_t get_buffer_length();
} outbound_pkt_ref;

class inbound_packet {
public:
    const uint8_t startByte1 = 0xBF;
    const uint8_t startByte2 = 0xFF;
private:
    uint8_t cmd_id;
    uint8_t data[255];
    uint8_t checksum;
}inbound_pkt_ref;


byte last_received_byte = 0;
bool last_byte_valid = false;

HardwareSerial serial_host_comms = SERIAL_COMMS;

void initSerialComms();
void sendData(uint8_t* data, size_t length);
bool receiveValidPacket();

uint8_t calculateChecksum(uint8_t* data, size_t length);