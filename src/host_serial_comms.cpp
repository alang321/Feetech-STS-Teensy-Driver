#include "host_serial_comms.h"
#include <packet_data_handlers.h>

void initSerialComms() {
    serial_host_comms.begin(SERIAL_COMMS_BAUDRATE);
}

void sendData(uint8_t* data, size_t length) {
    //todo make this conform to the packet structure
    not just data but also cmd_id
    outbound_pkt.checksum = calculateChecksum(data, length);

    do the data stuff here

    serial_host_comms.write(outbound_pkt.startByte1);
    serial_host_comms.write(outbound_pkt.startByte2);
    serial_host_comms.write(data, length);
    serial_host_comms.write(outbound_pkt.checksum);
}

uint8_t calculateChecksum(uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    calculate a proper checksum here
    for (int i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

bool receiveValidPacket(){
    if(serial_host_comms.available() > 0){
        byte next_byte = serial_host_comms.read();

        if(next_byte == 0xBF){
            last_received_byte = next_byte;
            last_byte_valid = true;
            return false;
        }
    }

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

