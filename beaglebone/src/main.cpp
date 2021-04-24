/* main.cpp
 *
 * TARS-MK1 beaglebone flight software
 *
 * Illinois Space Society - IREC 2021 Avioinics Team
 *
 * Ayberk Yaraneri
 *
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "../lib/rfm95/rfm95.h"
#include "../lib/uart/uart.h"

int main(void)
{

    // RFM95 rfm95;
    // rfm95.RFM_test();

    UART acUART;
    acUART.uart_init("/dev/ttyO1");

    // uint8_t data[71] = {0xDE, 0xAD, 0xBE, 0xEF};

    printf("sizeof(dataPacket_t) = %d", sizeof(dataPacket_t));

    while (1) {

		dataPacket_t* packet = acUART.uart_readPacket();

		if (packet == NULL) {
			printf("Packet is NULL\n");
			continue;
		}
		else {
			printf("Packet received!\n");
		}
		printf("ax:\t %f\n", packet->ax);
		printf("ay:\t %f\n", packet->ay);
		printf("az:\t %f\n", packet->az);
		printf("gx:\t %f\n", packet->gx);
		printf("gy:\t %f\n", packet->gy);
		printf("gz:\t %f\n", packet->gz);
		printf("mx:\t %f\n", packet->mx);
		printf("my:\t %f\n", packet->my);
		printf("mz:\t %f\n", packet->mz);
		printf("hg_ax:\t %f\n", packet->hg_ax);
		printf("hg_ay:\t %f\n", packet->hg_ay);
		printf("hg_az:\t %f\n", packet->hg_az);

		printf("Altitude:\t %f\n", packet->altitude);
		printf("latitude:\t %f\n", packet->latitude);
		printf("longitude:\t %f\n", packet->longitude);
		// printf("posLock:\t %d\n", packet->posLock);
		// printf("rocketState:\t %d\n", packet->rocketState);
		// printf("pt1:\t\t %f\n", packet->pt1);
		// printf("pt2:\t\t %f\n", packet->pt2);
		// printf("pt3:\t\t %f\n", packet->pt3);
		printf("timestamp:\t %d\n", packet->timeStamp);

		// Perform raw read
		// int nBytes = acUART.uart_read(data, 68);
		// printf("Received %d bytes:\n", nBytes);

		// Display raw bytes of received packet (sans sentinel bytes)
		
		uint8_t* data = (uint8_t*) packet;
		for (int i = 0; i < sizeof(dataPacket_t); ++i) {
			printf("0x%.2X\t", data[i]);
		}
		printf("\n\n");

    }


    return 0;
}
