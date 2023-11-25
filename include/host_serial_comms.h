#pragma once

#include "arduino.h"
#include <config.h>
#include <packet_data_structs.h>

class outbound_packet {
private:
    static const uint8_t startByte1 = 0xBF;
    static const uint8_t startByte2 = 0xFF;
    uint8_t* data; //here the data includes a reply_id
    uint8_t data_length;
    uint8_t checksum;

    uint8_t calculate_checksum();
public:
    outbound_packet(uint8_t* data, uint8_t data_length);
    uint8_t* get_buffer();
    uint8_t get_buffer_length();
};

class inbound_packet {
private:
    uint8_t received_checksum;
    uint8_t* data;
    uint8_t checksum;
    uint8_t cmd_id;

    uint8_t calculate_checksum();
    uint8_t get_data_and_checksum_length(); //length of the data and checksum
public:
    static const uint8_t startByte1 = 0xBF;
    static const uint8_t startByte2 = 0xFF;

    bool set_cmd_id(uint8_t cmd_id); //return false if the cmd_id is invalid
    bool set_data_and_checksum(uint8_t* data_and_checksum); //chops off the checksum

    uint8_t get_total_length(); //length of the packet including the checksum and start sequence
    uint8_t get_data_length(); //length of the data
    uint8_t* get_data();
    uint8_t get_cmd_id();
};

//used for detection of starting sequence
byte last_received_byte = 0;

int depth = 0;
//buffer to hold all the read bytes, size if the max size of the packet, the +1 is for the checksum
byte buffer[sizeof(cmd_union) + 1];

inbound_packet inbound_pkt;

HardwareSerial serial_host_comms = SERIAL_COMMS;

void initSerialComms();
void sendData(uint8_t* data, size_t length);
bool receiveValidPacket();

uint8_t calculateChecksum(uint8_t* data, size_t length);