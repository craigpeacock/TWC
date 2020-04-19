/*
Tesla Wall Connector Example
Copyright (C) 2020 Craig Peacock

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 3
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <stdio.h> 
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdint.h>
#include <stdbool.h>
#include <byteswap.h>

#pragma pack(1)

struct HEARTBEAT {
	uint8_t		startbyte;
	uint16_t	function;
	uint16_t	TWCID;
	uint32_t	totalkWh;
	uint16_t	phase_a_volts;
	uint16_t	phase_b_volts;
	uint16_t	phase_c_volts;
};

struct PACKET {
	uint8_t		startbyte;
	uint16_t	function;
	uint16_t	TWCID;
	uint8_t		payload_byte_0;
	uint8_t		payload_byte_1;
	uint8_t		payload_byte_2;
	uint8_t		payload_byte_3;
	uint8_t		payload_byte_4;
	uint8_t		payload_byte_5;
	uint8_t		payload_byte_6;
	uint8_t		payload_byte_7;
	uint8_t 	checksum;
	uint8_t		stopbyte;
};

struct FIRMWARE {
	uint8_t		startbyte;
	uint16_t	function;
	uint8_t		major;
	uint8_t		minor;
	uint8_t		revision;
	uint8_t		Pad_Byte_0;
	uint8_t		Pad_Byte_1;
	uint8_t		Pad_Byte_2;
	uint8_t		Pad_Byte_3;
	uint8_t		Pad_Byte_4;
	uint8_t		Pad_Byte_5;
	uint8_t		Pad_Byte_6;
	uint8_t		Pad_Byte_7;
	uint8_t		checksum;
	uint8_t		stopbyte;
};

bool DecodeHeartBeat(struct HEARTBEAT *HeartBeat)
{
	printf("Master HeartBeat: ");
	printf("Total kWh since build : %lukWh, ", (long unsigned int)bswap_32(HeartBeat->totalkWh));
	printf("Voltage Phase A : %dV, ", bswap_16(HeartBeat->phase_a_volts));
	printf("Phase B : %dV, ", bswap_16(HeartBeat->phase_b_volts));
	printf("Phase C : %dV\r\n", bswap_16(HeartBeat->phase_c_volts));
	return(true);
}

bool DecodeFirmware(struct FIRMWARE *FirmwareVer)
{
	printf("Firmware Version %d.%d.%d\r\n", FirmwareVer->major, FirmwareVer->minor, FirmwareVer->revision);
	return(true);
}

bool PrintPacket(uint8_t *buffer, uint8_t nbytes)
{
	uint8_t i;
	printf("> %d bytes:",nbytes);
	for (i = 0; i < nbytes; i++)
		printf(" %02X", (uint8_t)buffer[i]);
	printf("\r\n");	
}

bool VerifyCheckSum(uint8_t *buffer, uint8_t nbytes)
{
	uint8_t i;
	uint8_t endbyte = 0;
	uint8_t checksum = 0;
	
	// Message should always begin with a 0xC0 start byte	
	if (buffer[0] != 0xC0) printf("Start byte incorrect\r\n");
	
	// Now find the endbyte
	for (i = 1; i < nbytes; i++) {
		if ((unsigned char)buffer[i] == 0xC0) endbyte = i; 
	}
	//printf("Endbyte = %d",endbyte);

	// Calculate the checksum. 
	// The checksum is the sum of all the databytes excluding the first byte after the start byte
	for (uint8_t i = 2; i < endbyte - 2; i++) {
		checksum = checksum + (unsigned char)buffer[i];
	}
	//printf("Checksum = %02X\r\n",checksum);
	
	if ((unsigned char)buffer[endbyte -1] == checksum) 
		return true;
	else
		return false;
}

uint8_t CalculateCheckSum(uint8_t *buffer, uint8_t nbytes)
{
	uint8_t i;
	uint8_t endbyte = 0;
	uint8_t checksum = 0;
	
	// Message should always begin with a 0xC0 start byte	
	if (buffer[0] != 0xC0) printf("Start byte incorrect\r\n");
	
	// Now find the endbyte
	for (i = 1; i < nbytes; i++) {
		if ((unsigned char)buffer[i] == 0xC0) endbyte = i; 
	}
	//printf("Endbyte = %d",endbyte);

	// Calculate the checksum. 
	// The checksum is the sum of all the databytes excluding the first byte after the start byte
	for (uint8_t i = 2; i < endbyte - 2; i++) {
		checksum = checksum + (unsigned char)buffer[i];
	}
	//printf("Checksum = %02X\r\n",checksum);

	return checksum;
}

bool ProcessPacket(uint8_t *buffer, uint8_t nbytes)
{
	struct PACKET *packet = (struct PACKET *)buffer;
	//printf("Processing Packet");
	
	switch (bswap_16(packet->function)) {
		case 0xFDEB:	
			DecodeHeartBeat((struct HEARTBEAT *)buffer);
			break;
		case 0xFDE2:
			//DecodeLinkReady((LINKREADY *)buffer);
			break;
		case 0xFD1B:
			DecodeFirmware((struct FIRMWARE *)buffer);
			break;
		default:
			break;
	}
}

int GetFirmwareVersion(int fd)
{
	int nbytes; 
	struct PACKET packet;
	
	packet.startbyte = 0xC0;
	packet.function = bswap_16(0xFB1B);
	packet.TWCID = bswap_16(0x9819);
	packet.payload_byte_0 = 0x00;
	packet.payload_byte_1 = 0x00;
	packet.payload_byte_2 = 0x00;
	packet.payload_byte_3 = 0x00;
	packet.payload_byte_4 = 0x00;
	packet.payload_byte_5 = 0x00;
	packet.payload_byte_6 = 0x00;
	packet.payload_byte_7 = 0x00;
	packet.stopbyte = 0xC0;
	
	packet.checksum = CalculateCheckSum((uint8_t *)&packet, sizeof(packet));
	PrintPacket((uint8_t *)&packet, sizeof(packet));
	if ((nbytes = write(fd, (uint8_t *)&packet, sizeof(packet))) < 0) {
		perror("write");
	} else {
		printf("wrote %d bytes\r\n",nbytes);
	} 
}

int main(int argc, char **argv)
{
	int fd; 	
	char buffer[64]; 
	
	int nbytes; 
	int i; 
	struct termios options;
	char *field[20];
		
	if ((fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
		perror("Open");
		return 1;
	} 
	
	// Set to blocking
	fcntl(fd, F_SETFL, 0); 

	// Get port attributes
	tcgetattr(fd, &options);

	// Set input and output baud rates
	cfsetispeed(&options, B9600);
	cfsetospeed(&options, B9600);

	// Clear all input modes
	options.c_iflag = 0;
	
	// Set 8 bits, no parity, 1 stop bit
	options.c_cflag &= ~PARENB;	// Clear parity generation
	options.c_cflag &= ~CSTOPB;	// Clear two stop bits
	options.c_cflag &= ~CSIZE;	// Clear Character size mask
	options.c_cflag |= CS8;		// Set Character mask to 8 bits
	options.c_cflag | CLOCAL;	// Ignore modem control lines
	
	options.c_lflag &= ~ECHO;
	
	options.c_lflag &= ~ICANON;	// Disable canonical mode
	options.c_cc[VMIN]=0;		// Minimum number of characters in noncanonical mode
	options.c_cc[VTIME]=0;		// timeout in deciseconds
	
	// Set port attributes
	tcsetattr(fd, TCSAFLUSH, &options);
		
	printf("Port Opened\r\n");

	GetFirmwareVersion(fd);
	
	do {
		if ((nbytes = read(fd, &buffer, sizeof(buffer))) < 0) {
			perror("Read");
			return 1;
		} else {
			if (nbytes == 0) {
				printf(".\r\n");
				//printf("No communication from Tesla Wall Connector\r\n");
				sleep(1);
			} else {
				PrintPacket(buffer, nbytes);
				if (VerifyCheckSum(buffer, nbytes)) ProcessPacket(buffer, nbytes);
				//sleep(1);
				//GetFirmwareVersion(fd);
			}
		}
	} while(1);

	if (close(fd) < 0) {
		perror("Close");
		return 1;
	}

	return (0);
}