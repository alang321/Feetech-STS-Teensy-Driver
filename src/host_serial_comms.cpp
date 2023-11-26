#include "host_serial_comms.h"
#include <packet_data_handlers.h>
#include <packet_data_structs.h>

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
        //read the next byte
        uint8_t next_byte = serial_host_comms.read();
        
        //Do different things depending on the depth into the packet, 
        //depth 0 check for start sequence, depth 2 check the command id, depth 3 and onwards read the data and check the checksum
        //If the command id is not valid then depth is set to 0
        //If the checksum is not valid then depth is set to 0
        //If an early start sequence is found in the data then depth is set to 2
        switch (depth)
        {
            case 0:
                if(next_byte == inbound_packet::startByte2 && last_received_byte == inbound_packet::startByte1){
                    depth = 2;
                    #ifdef DEBUG
                    Serial.println("Start sequence found.");
                    #endif
                }
                break;
            case 2:
                //check if the command id is valid
                if(inbound_pkt.set_cmd_id(next_byte)){
                    depth++;
                    #ifdef DEBUG
                    Serial.println("Valid command id found.");
                    Serial.print("Command id: ");
                    Serial.println(next_byte);
                    #endif
                }else{
                    #ifdef DEBUG
                    Serial.println("Invalid command id, aborting packet read.");
                    Serial.print("Command id: ");
                    Serial.println(next_byte);
                    #endif
                    depth = 0;
                }
                break;
            default:
                depth++;
                
                //read the next byte into the buffer
                buffer[depth - 4] = next_byte;

                //check if we are reached the end of the data/checksum section of the packet
                if(depth == inbound_pkt.get_total_length()){
                    depth = 0;
                    //set the data and checksum in the inbound packet, if the checksum is invalid then abort the packet read
                    if(inbound_pkt.set_data_and_checksum(buffer)){
                        //call the correct handler
                        #ifdef DEBUG
                        Serial.println("Valid packet received.");
                        #endif
                        serial_cmd_handlers[inbound_pkt.get_cmd_id()](inbound_pkt.get_data());
                        last_received_byte = 0;
                        return true;
                    }else{
                        #ifdef DEBUG
                        Serial.println("Invalid checksum, aborting packet read.");
                        Serial.print("Received checksum: ");
                        Serial.println(inbound_pkt.received_checksum);
                        Serial.print("Calculated checksum: ");
                        Serial.println(inbound_pkt.calculate_checksum());
                        #endif
                    }
                }

                //check if there is a start sequence in the data, if there is then abort the packet read
                //dont check if the nextbyte was a valid checksum byte
                //because here there might be an unintended start sequence in the data, extremely unlikely but possible if the checksum is 0xFF
                //and the previous byte was 0xBF
                if(next_byte == inbound_packet::startByte2 && last_received_byte == inbound_packet::startByte1){
                    depth = 2;
                    #ifdef DEBUG
                    Serial.println("Unexpected early start sequence found in data, aborting packet read.");
                    #endif
                }
                break;
        }   

        last_received_byte = next_byte;
    }
    return false;
}

outbound_packet::outbound_packet(uint8_t* data, uint8_t data_length) {
    this->data = data;
    this->data_length = data_length;
    this->checksum = calculate_checksum();
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

uint8_t outbound_packet::calculate_checksum() {
    uint16_t sum = 0;

    for (size_t i = 0; i < data_length; ++i) {
        sum += data[i];
    }

    return ~static_cast<uint8_t>(sum % 256);
}


bool inbound_packet::set_cmd_id(uint8_t cmd_id) {
    if(cmd_id <= cmd_id_max){
        this->cmd_id = cmd_id;
        return true;
    }else{
        return false;
    }
}

uint8_t inbound_packet::calculate_checksum() {
    uint16_t sum = 0;

    sum += cmd_id;

    for (size_t i = 0; i < get_data_length(); ++i) {
        sum += data[i];
    }

    return ~static_cast<uint8_t>(sum % 256);
}

uint8_t inbound_packet::get_data_and_checksum_length() {
    return get_data_length() + 1;
}

bool inbound_packet::set_data_and_checksum(uint8_t* data_and_checksum) {
    //check if the checksum is valid
    received_checksum = data_and_checksum[get_data_and_checksum_length() - 1];
    if(received_checksum == calculate_checksum()){
        //free the old data
        delete[] data;

        //check if the data length is 0, if it is then set the data to nullptr
        //maybe this is not needed, but I dont wanna test
        if(get_data_length() == 0){
            data = nullptr;
            return true;
        }

        //copy the data
        data = new uint8_t[get_data_length()];
        memcpy(data, data_and_checksum, get_data_length());
        return true;
    }else{
        return false;
    }
}

uint8_t inbound_packet::get_total_length() {
    return get_data_and_checksum_length() + 2;
}

uint8_t inbound_packet::get_data_length() {
    return cmd_lengths[cmd_id];
}

uint8_t* inbound_packet::get_data() {
    return data;
}

uint8_t inbound_packet::get_cmd_id() {
    return cmd_id;
}

