#include "host_serial_comms.h"
#include <packet_data_handlers.h>

void initSerialComms() {
    serial_host_comms.begin(SERIAL_COMMS_BAUDRATE);
}

void sendData(uint8_t* data, size_t length) {
    //todo make this conform to the packet structure
    outbound_packet outbound_pkt(data, length);

    serial_host_comms.write(outbound_pkt.get_buffer(), outbound_pkt.get_buffer_length());
}



bool receiveValidPacket(){
    if(serial_host_comms.available() > 0){
        byte next_byte = serial_host_comms.read();

        //check if this is the start of a packet
        if(last_byte_valid && last_received_byte == inbound_pkt_ref.startByte1 && next_byte == inbound_pkt_ref.startByte2){
            last_byte_valid = false;

            //read the command id
            SERIAL_COMMS.readBytes((char*) &inbound_pkt_ref.cmd_id, 1);	

            //read the data into correct struct

            //check if the packet is valid

            //call the correct handler

            return true;
        }

        last_byte_valid = true;
        last_received_byte = next_byte;
    }
    return false;

    //read byte


    //check if read byte and last received byte are correct starting SERIAL_SERVOS_BAUDRATE

    //if yes then go on to reading packet and checking cheksum, also so last read byte to null or set some flag

    //if not then set last read byte to current read byte

    
    //last vreceived byte is this






    
    if(SERIAL_COMMS.available() > 0)
    {
    //read the first byte in the serial buffer
    byte first_byte;
    SERIAL_COMMS.readBytes((char*) &first_byte, 1);
    //check if it is the start byte
    if(first_byte != 0xBF)
        return;

    //read the second byte in the serial buffer
    byte second_byte;
    SERIAL_COMMS.readBytes((char*) &second_byte, 1);

    //check if it is the second start byte
    if(second_byte == 0xFF){
        #ifdef DEBUG
        unsigned long current_time = millis();

        unsigned long time_since_last_message = current_time - last_time;
        Serial.print("Received start marker, time since last message: ");
        Serial.println(time_since_last_message);

        last_time = current_time;

        #endif

        #ifdef DEBUG
        Serial.println("received message");
        #endif

        // Read the command id
        uint8_t cmd_id = 0;
        SERIAL_COMMS.readBytes((char*) &cmd_id, 1);

        #ifdef DEBUG
        Serial.print("cmd_id: ");
        Serial.println(cmd_id);
        #endif

        //call the correct handler
        serial_cmd_handlers[cmd_id]();
        }
    }
}

outbound_packet::outbound_packet(void* data, uint8_t data_length) {
    this->data = data;
    this->data_length = data_length;
    this->checksum = calculateChecksum();
}

uint8_t* outbound_packet::get_buffer() {
    uint8_t* buffer = new uint8_t[get_buffer_length()];
    //starting seqeunce
    buffer[0] = startByte1;
    buffer[1] = startByte2;
    //data
    memcpy(buffer + 2, data, data_length);
    //checksum
    buffer[data_length + 2] = checksum;
    return buffer;
}

uint8_t outbound_packet::get_buffer_length() {
    return data_length + 3;
}

uint8_t outbound_packet::calculateChecksum() {
    // crc 1 byte checksum
    uint8_t checksum = 0;
    for (int i = 0; i < data_length; i++) {
        checksum += ((uint8_t*) data)[i];
    }
    return checksum;
}

