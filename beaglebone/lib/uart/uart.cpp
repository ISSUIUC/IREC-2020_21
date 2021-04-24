/* uart.cpp
 *
 * UART wrapper library that abstracts linux ioctls
 *
 * Illinois Space Society - IREC 2021 Avioinics Team
 *
 * Ayberk Yaraneri
 *
 */

 #include <stdint.h>
 #include <stdio.h>
 #include <string.h>

 // Linux headers
 #include <fcntl.h> // Contains file controls like O_RDWR
 #include <errno.h> // Error integer and strerror() function
 #include <termios.h> // Contains POSIX terminal control definitions
 #include <unistd.h> // write(), read(), close()

 #include "uart.h"


 /* UART::uart_init
  * DESCRIPTION:    opens file for uart hardware and performs initialization
  * INPUTS:         port -- linux port filename
  * RETURNS:        linux file descriptor on success, -1 on failure
  * 
  * Look at linux man pages for the termios library for explanations of fields
  * https://man7.org/linux/man-pages/man3/termios.3.html
  */
int UART::uart_init(char* port) {

    _fd = open(port, O_RDWR);

    struct termios tty;

    if(tcgetattr(_fd, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    // tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VTIME] = 0;     // Wait indefinitely until the minimum number of bytes have been read.
    tty.c_cc[VMIN] = 71;     // Only return when this minimum number of bytes are received.

    // Set in/out baud rate to be 9600. NEEDS SAME BAUD RATE AS TEENSY
    // Remember we will need to test for the optimal speed once the teensy's are set up.
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(_fd, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }

	/* Internal fields for packet detection */
	packet_bytes_received = 0;
	sentinel_detected = false;
	fsm_state = 0;

    return _fd;
}

/* UART::uart_read
 * DESCRIPTION:     reads bytes from UART rx
 * INPUTS:          buf -- buffer to populate with read bytes
 *                  numBytes -- number of bytes to read
 * RETURNS:         number of bytes read, -1 on failure
 */
int32_t UART::uart_read(uint8_t* buf, uint32_t numBytes) {
    int32_t ret;
    ret = read(_fd, buf, numBytes);
    if (ret < 0) {
        printf("Error reading to UART: %s", strerror(errno));
        return -1;
    }
    return ret;
}

/* UART::uart_write
 * DESCRIPTION:     writes bytes to UART tx
 * INPUTS:          buf -- buffer to write from
 *                  numBytes -- number of bytes to write
 * RETURNS:         number of bytes written, -1 on failure
 */
int32_t UART::uart_write(uint8_t* buf, uint32_t numBytes) {
    int32_t ret;
    ret = write(_fd, buf, numBytes);
    if (ret < 0) {
        printf("Error writing to UART: %s", strerror(errno));
        return -1;
    }
    return ret;
}

/* UART::uart_readPacket
 * DESCRIPTION:     reads a dataPacket from the uart input buffer, assembles
 *					a dataPacket struct
 * INPUTS:          none
 * RETURNS:         pointer to assembled dataPacket struct,
 *					NULL if no dataPacket is asembled yet
 */
dataPacket_t* UART::uart_readPacket() {

	uart_read(input_buf, INPUT_BUF_SIZE);

	int32_t startIdx = -1;
	if (sentinel_detected == false) {
		startIdx = sentinel_detect();
		if (startIdx < 0) return NULL;
	}
	printf("%ld\n",startIdx);

	for (int32_t idx = startIdx; idx < INPUT_BUF_SIZE; ++idx) {
		packetBuf[packet_bytes_received] = input_buf[idx];
		++packet_bytes_received;

		if (packet_bytes_received == PACKET_SIZE) {
			packet_bytes_received = 0;
			sentinel_detected = false;
			// for (int i = 0; i < sizeof(packetBuf); ++i) {
			// 	printf("0x%.2X\t",packetBuf[i]);
			// }
			// printf("\n");
			return (dataPacket_t*) packetBuf;
		}
	}

	return NULL;
}

/* UART::uart_write
 * DESCRIPTION:     Finds a sequence of sentinel bytes in uart input buffer
 *					using a simple finite state machine
 * INPUTS:          none
 * RETURNS:         index after sentinel bytes, returns -1 if no sentinel
 *					sequence is detected or on error
 */
int32_t UART::sentinel_detect() {

	for (uint32_t i = 0; i < INPUT_BUF_SIZE; ++i) {
		switch(fsm_state) {
			case 0:
				if (input_buf[i] == SENTINEL_1) {
					fsm_state = 1;
				}
				break;
			case 1:
				if (input_buf[i] == SENTINEL_2) {
					fsm_state = 2;
				}
				else {
					fsm_state = 0;
				}
				break;
			case 2:
				if (input_buf[i] == SENTINEL_3) {
					fsm_state = 0;
					sentinel_detected = true;
					printf("##### SENTINEL DETECTED #####\n");
					if (i == INPUT_BUF_SIZE-1) {
						return 0;
					}
					else {
						return i+1;
					}
				}
				else {
					fsm_state = 0;
				}
			default:
				printf("UART PacketFifo Error!\n");
				return -1;
		}

	}
	return -1;

}
