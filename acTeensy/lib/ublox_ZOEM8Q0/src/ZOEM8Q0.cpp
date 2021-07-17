#include "ZOEM8Q0.hpp"
#include "Arduino.h"
#include "SPI.h"

//#define GPS_SERIAL_DEBUG

//Constructor
ZOEM8Q0::ZOEM8Q0() {
    //Do any necessary setup
    position_lock = false;
    latitude = 0;
    longitude = 0;
    altitude = 0;
}

void ZOEM8Q0::beginSPI(uint16_t CS_pin) {
    pinMode(CS_pin, OUTPUT);
    SPI.begin();
    initSPI(CS_pin);
}

void ZOEM8Q0::initSPI(uint16_t CS_pin) {
    //The maximum transfer rate using SPI is 125 kB/s and the
    //maximum SPI clock frequency is 5.5 MHz.
    SPI.setClockDivider(SPI_CLOCK_DIV64);

    // Data is read and written MSb first.
    SPI.setBitOrder(MSBFIRST);

    // Clock starting position is low.
    // Data is recieved on falling edge of clock
    SPI.setDataMode(SPI_MODE0);
}

void ZOEM8Q0::endSPI() {
    SPI.end();
}

bool ZOEM8Q0::update_data() {   
    return process_GPS_NMEA(ZOEM8Q0_CS);
}

bool ZOEM8Q0::process_GPS_NMEA(uint16_t CS_pin) {
    //buffer for XXGGA data
    uint8_t buffer[MAX_BUFFER_SIZE];

    NMEA_parser_state parser_state = SEARCH_START;

    // select GPS chip
    digitalWrite(CS_pin, LOW);

    SPI.transfer(0x80 | 0xA2);

    int buffer_idx = 0;

    for (int counter = 0; counter < MAX_BUFFER_SIZE; ++counter) {

        buffer[buffer_idx] = SPI.transfer(0x00);

        if (buffer_idx > 0 && buffer[buffer_idx-1] == 0xFF && buffer[buffer_idx] == 0xFF) {
            digitalWrite(CS_pin, HIGH);
            parser_state = SEARCH_START;
            return false;
        }

        switch (parser_state) {

            case SEARCH_START:

                ++buffer_idx;

                if (buffer[0] == START_CHAR) {
                    parser_state = START_DETECTED;
#ifdef GPS_SERIAL_DEBUG
                    Serial.println("START_DETECTED");
#endif
                } else {
                    buffer_idx = 0;
                }

                break;

            case START_DETECTED:

                ++buffer_idx;

                if (buffer_idx == 6) {

#ifdef GPS_SERIAL_DEBUG
                    Serial.println("##### HEADER ACQUIRED #####");
#endif

                    uint32_t header = (buffer[3] << 16) |
                                      (buffer[4] << 8) |
                                      buffer[5];

                    if (header == GGA_MSG) {
                        parser_state = GGA_DETECTED;
#ifdef GPS_SERIAL_DEBUG
                        Serial.println("##### GGA_DETECTED #####");
#endif
                    } else {
                        buffer_idx = 0;
                        parser_state = SEARCH_START;
                    }
                }
                break;

            case GGA_DETECTED:

                ++buffer_idx;

                if (buffer[buffer_idx-2] == 0x0D && buffer[buffer_idx-1] == 0x0A) {
                    parser_state = END_DETECTED;
#ifdef GPS_SERIAL_DEBUG
                        Serial.println("##### END_DETECTED #####");
#endif
                }
                break;
        }

        if (parser_state == END_DETECTED) break;

    }

    digitalWrite(CS_pin, HIGH);
    
#ifdef GPS_SERIAL_DEBUG
    //Printing XXGGA Message if it is detected
    for (int i = 0; i < buffer_idx; ++i) {
        Serial.print((char) buffer[i]);
    }
#endif
   
    if (parser_state == END_DETECTED) {
        position_lock = decode_xxgga_sentence(buffer, buffer_idx);
#ifdef GPS_SERIAL_DEBUG
        Serial.println("XXGGA MESSAGE PARSED SUCCESSFULLY");
#endif 
        if (position_lock) {
#ifdef GPS_SERIAL_DEBUG
            Serial.println("GPS POSITION FOUND"); 
#endif
            convert_data();
        } else {
#ifdef GPS_SERIAL_DEBUG
            Serial.println("Could not lock onto GPS coordinates");             
#endif
        }
    }

    return (parser_state == END_DETECTED);

}

char* ZOEM8Q0::arr_substring(uint8_t* buffer, int start, int end) {
    int len = end - start;
    char* data = new char[len];
    for (int i = 0; i < len; i++) {
        data[i] = buffer[start + i];
    }
    return data;
}

bool ZOEM8Q0::decode_xxgga_sentence(uint8_t *buffer, uint16_t dataLength){

#ifdef GPS_SERIAL_DEBUG
    Serial.println("-----Decoding XXGGA Message-----");        
#endif
    
    int delim_count = 0;

    int lat_start = 0;
    int lat_end = 0; //non inclusive
    bool isLatSt = false;
    bool isLatEnd = false;

    int lon_start = 0;
    int lon_end = 0;
    bool isLonSt = false;
    bool isLonEnd = false;

    int ht_start = 0;
    int ht_end = 0;
    bool isHtSt = false;
    bool isHtEnd = false;

    for (int i = 0; i < dataLength; i++) {
        char character = (char)*(buffer + i);
        if (delim_count == 2 && character != ',' && !isLatSt) {
            lat_start = i;
            isLatSt = true;
        } else if (delim_count == 3 && character != ',' && !isLatEnd) {
            lat_end = i - 1;
            nmea_data.lat_direction = character;
            isLatEnd = true;
        } else if (delim_count == 4 && character != ',' && !isLonSt) {
            lon_start = i;
            isLonSt = true;
        } else if (delim_count == 5 && character != ',' && !isLonEnd) {
            lon_end = i - 1;
            nmea_data.lon_direction = character;
            isLonEnd = true;
        } else if (delim_count == 6 && character == '0') {
            //Position fix is 0 and there is no lock on the position.
            return false;
        } else if (delim_count == 9 && character != ',' && !isHtSt) {
            ht_start = i;
            isHtSt = true;
        } else if (delim_count == 10 && character != ',' && !isHtEnd) {
            ht_end = i - 1;
            isHtEnd = true;
        } else if (character == ',') {
            delim_count++;
        } else {
            continue;
        }
    }
    char* lat = arr_substring(buffer, lat_start, lat_end);
    nmea_data.std_lat = (float)atof(lat);
    delete[] lat;

    char* lon = arr_substring(buffer, lon_start, lon_end);
    nmea_data.std_lon = (float)atof(lon);
    delete[] lon;

    char* ht = arr_substring(buffer, ht_start, ht_end);
    nmea_data.height = (float)atof(ht);
    delete[] ht;

    //position fix is anything other than 0 since we made it though the loop without returning false
    return true;
}


void ZOEM8Q0::convert_data() {
    int north_south = 1;
    int east_west = 1;
    //If the latitude direction is south, make latitude negative
    if (nmea_data.lat_direction == 0x53) {
        north_south = -1;
    }
    //If the longitude direction is west, make longitude negative
    if (nmea_data.lon_direction == 0x57) {
        east_west = -1;
    }
    int lat_deg = nmea_data.std_lat/100;
    float lat_min = nmea_data.std_lat - lat_deg * 100;
    int lon_deg = nmea_data.std_lon/100;
    float lon_min = nmea_data.std_lon - lon_deg * 100;
    //converting lat/lon to normal standards of degrees
    //N and E are positive by standards. S and W are considered negative
    latitude = north_south*(lat_deg + (lat_min/60.0));
    longitude = east_west*(lon_deg + (lon_min/60.0));
    altitude = nmea_data.height;

#ifdef GPS_SERIAL_DEBUG
    Serial.println("Converted Data: ");
	Serial.print("Latitude: ");
	Serial.println(latitude);
	Serial.print("Longitude: ");
	Serial.println(longitude);
	Serial.print("Altitude: ");
	Serial.println(altitude);
#endif
}


float ZOEM8Q0::get_latitude() {
    return latitude;
}

float ZOEM8Q0::get_longitude() {
    return longitude;
}

float ZOEM8Q0::get_altitude() {
    return altitude;
}

bool ZOEM8Q0::get_position_lock() {
    return position_lock;
}
