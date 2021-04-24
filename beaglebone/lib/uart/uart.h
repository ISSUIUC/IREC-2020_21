/* uart.h
 *
 * UART wrapper library that abstracts linux ioctls
 *
 * Illinois Space Society - IREC 2021 Avioinics Team
 *
 * Ayberk Yaraneri
 *
 */

#include <stdint.h>


#define SENTINEL_1 0x49     // ascii I
#define SENTINEL_2 0x53     // ascii S
#define SENTINEL_3 0x53     // ascii S

#define INPUT_BUF_SIZE  128
#define OUTPUT_BUF_SIZE 128

enum FSM_State {
    STATE_INIT,
    STATE_IDLE,
    STATE_LAUNCH_DETECT,
    STATE_BOOST,
    STATE_BURNOUT_DETECT,
    STATE_COAST,
    STATE_APOGEE_DETECT,
    STATE_APOGEE,
    STATE_DROGUE,
    STATE_MAIN
};


typedef struct dataPacket_t {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;

    //! data for highGimu accel data (hg_x, hg_y, hg_z)
    float hg_ax;
    float hg_ay;
    float hg_az;
    // int16_t pt1;
    // int16_t pt2;
    // int16_t pt3;

    //GPS DATA
    float latitude;
    float longitude;
    float altitude;
    // bool posLock;

    // FSM_State rocketState;
    int32_t timeStamp;

};

#define PACKET_SIZE		sizeof(dataPacket_t)

class UART {
    public:

	    int uart_init(char* port);

		int32_t uart_read(uint8_t* buf, uint32_t numBytes);
		int32_t uart_write(uint8_t* buf, uint32_t numBytes);

		dataPacket_t* uart_readPacket();
		int32_t sentinel_detect();

    private:

        /* UART interface */
        //int uart_init(char* port);

        int _fd;    // Linux file descriptor

        uint8_t input_buf[INPUT_BUF_SIZE];
        uint8_t output_buf[OUTPUT_BUF_SIZE];

        /* Packet detection and assembly */
		uint8_t	packetBuf[PACKET_SIZE];
		uint32_t packet_bytes_received;

		bool sentinel_detected;
		uint8_t fsm_state;
};
