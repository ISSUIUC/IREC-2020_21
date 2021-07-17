/* ZOEM8Q0.hpp
 *
 * U-Blox ZOEM8Q0 GPS driver for TARS-MK1
 * 
 * Utilizes SPI communication and the standard NMEA protocol to find the latitude, longitude, and altitude.
 * Data Sheet: https://www.u-blox.com/sites/default/files/ZOE-M8_DataSheet_%28UBX-16008094%29.pdf
 * Interface Manual: https://www.u-blox.com/en/docs/UBX-13003221
 *
 * Illinois Space Society - IREC 2021 Avioinics Team
 *
 * Matt Taylor and Ayberk Yaraneri
 *
 */


#include "Arduino.h"

/******************************************************************************/
/* ZOEM8Q0 CLASS DEFINITION */

class ZOEM8Q0 {
    /* Chip select pin and (somewhat) Register Definition for ZOE-M8Q-0 */
    #define ZOEM8Q0_CS 34
    #define ZOEM8Q0_WHOAMI_REG 0xA2

    /* Current latitude in degrees */
    float latitude;
    /* current longitude in degrees */
    float longitude;
    /* height above mean sea level in m */
    float altitude;
    /* True if GPS has a lock on the position */
    bool position_lock;

    /* Struct for XXGGA NMEA data recieved
    * Contains:
    * float std_lat          :     Standatd for latitude (in degrees + minutes)
    * uint8_t lon_direction  :     Direction for latitude (North or South)
    * float std_lon          :     Standatd for longitude (in degrees + minutes)
    * uint8_t lon_direction  :     Direction for longitude (East or West)
    * float height           :     Current height above sea level in m
    */
    struct NMEA_DATA {
        float std_lat;
        uint8_t lat_direction;
        float std_lon;
        uint8_t lon_direction;
        float height;
    };
    NMEA_DATA nmea_data;

public:
    /* Constructor. Initializes latitude, longitude, altitude, and position_lock */
    ZOEM8Q0();

    /* Begins SPI communication with gps module. */
    void beginSPI(uint16_t CS_pin);

    /* Ends SPI communication with gps module. */
    void endSPI();

    /* Updates latitude, longitude, and altitude */
    bool update_data();

    /* Gets the current latitude in degrees. 
     *
     * Note that by standard:
     * latitude is positive if N
     * latitude is negative if S
     * 
     */
    float get_latitude();

    /* Gets the current longitude in degrees. 
     *
     * Note that by standard:
     * longitude is positive if E
     * longitude is negative if W
     * 
     */    
    float get_longitude();

    /* Gets the current altitude in m. */
    float get_altitude();

    /* Flag for position lock. 
     * Returns true when accurate GPS data is available.
     */
    bool get_position_lock();

private:

    #define MAX_BUFFER_SIZE 512
    /* Start Character for XXGGA Message */
    #define START_CHAR      0x24

    /* GGA in ASCII */
    #define GGA_MSG         0x00474741

    /* NMEA Message End Sequence: <CR><LF> */
    #define END_SEQ         0x0D0A

    enum NMEA_parser_state {
        SEARCH_START,
        START_DETECTED,
        GGA_DETECTED,
        END_DETECTED
    };

    /* Initializes SPI communication with correct settings. (MSB first, etc) */
    void initSPI(uint16_t CS_pin);

    /* Converts the latitude and longitude into degree format and sets the global variables.*/
    void convert_data();


    /*
     *Finds lattitude, longitude and altitude using NMEA protocol with GPRMC and xxGGA sentences.
     *Returns true if there is a signal position lock and false otherwise.
     *
     * EXAMPLE:
     * "$GPGGA,16405.00,4005.42330,N,10511.10842,W,1,09,0.91,15840,M,-21.4,M,1.0,0000*53"
     * 
     * 2nd comma signals latitude is next (in degree + minute format) so 4005.42330 N is really 40 degrees + 05.42330/60 degrees = 45.09038833 N
     * 3rd comma signals N/S is next
     * 4th comma signals longitude is next (in degree + minute format) so 10511.10842 N is really 105 degrees + 11.10842/60 degrees = 105.1851403 W
     * 5th comma signals E/W
     * 6th comma signals the position fix quality
     *      0 = No fix, 1 = Autonomous GNSS fix, 2 = Differential GNSS fix, 4 =RTK fixed, 5 = RTK float, 6 = Estimated/Dead reckoning fix
     * 9th comma signals the altitude in m is next
     */
    bool process_GPS_NMEA(uint16_t CS_pin);

    /* Helper function in decode_xxGGa_sentence. Creates a substring within the char array. 
     * Returns a char pointer to the new substring
     */
    char* arr_substring(uint8_t* buffer, int start, int end);


    /*
     *Decodes the current xxgga_sentence separated by commas and fills lattitude, longitude, and altitude.
     *Returns true if there is a signal position lock and false otherwise.
     */
    bool decode_xxgga_sentence(uint8_t* buffer, uint16_t dataLength);

};
