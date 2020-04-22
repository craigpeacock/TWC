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
#include <stdlib.h>

// Commands from https://teslamotorsclub.com/tmc/posts/3225600/
// Commands with reponses (0xFB)
#define GET_SERIAL_NUMBER	0xFB19
#define GET_MODEL_NUMBER	0xFB1A
#define GET_FIRMWARE_VER 	0xFB1B
#define GET_PLUG_STATE		0xFBB4
#define GET_VIN_FIRST		0xFBEE
#define GET_VIN_MIDDLE		0xFBEF
#define GET_VIN_LAST		0xFBF1

// Commands without responses (0xFC)
#define START_CHARGING		0xFC1B
#define STOP_CHARGING		0xFCB2



#define LINKREADY1			0xFCE1
#define LINKREADY2			0xFCE2

// Responses (0xFD)
#define RESP_SERIAL_NUMBER	0xFD19
#define RESP_MODEL_NUMBER	0xFD1A
#define RESP_FIRMWARE_VER	0xFD1B
#define RESP_LINK_READY		0xFDE2	
#define RESP_HEART_BEAT		0xFDEB
#define RESP_VIN_FIRST		0xFDEE
#define RESP_VIN_MIDDLE		0xFDEF
#define RESP_VIN_LAST		0xFDF1

#pragma pack(1)

struct CIRCULAR_BUFFER {
	uint8_t * buffer;
	uint16_t head;
	uint16_t tail;
	uint16_t max;
	bool full;
};

struct HEARTBEAT {
	uint8_t		startframe;
	uint16_t	function;
	uint16_t	TWCID;
	uint32_t	totalkWh;
	uint16_t	phase_a_volts;
	uint16_t	phase_b_volts;
	uint16_t	phase_c_volts;
};

struct PACKET {
	uint8_t		startframe;
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
	uint8_t		endframe;
};

struct FIRMWARE {
	uint8_t		startframe;
	uint16_t	function;
	uint8_t		major;
	uint8_t		minor;
	uint8_t		revision;
	uint8_t		pad_byte_0;
	uint8_t		pad_byte_1;
	uint8_t		pad_byte_2;
	uint8_t		pad_byte_3;
	uint8_t		pad_byte_4;
	uint8_t		pad_byte_5;
	uint8_t		pad_byte_6;
	uint8_t		pad_byte_7;
	uint8_t		checksum;
	uint8_t		endframe;
};

struct STRING {
	uint8_t		startframe;
	uint16_t	function;
	uint8_t		string[11];
	uint8_t		checksum;
	uint8_t		endframe;
};

struct LINKREADY {
	uint8_t		startframe;			// Should be 0xC0
	uint16_t	function;			// 0xFCE1 LinkReady1 or 0xFCE2 LinkReady 2
	uint16_t	slaveTWCID;			// Tesla Wall Connector ID
	uint8_t	sign;				
	uint16_t	maxchargerate;
	uint8_t		payload_byte_0;		
	uint8_t		payload_byte_1;		
	uint8_t		payload_byte_2;		
	uint8_t		payload_byte_3;
	uint8_t		payload_byte_4;
	uint8_t		payload_byte_5;
	uint8_t		payload_byte_6;
	uint8_t		payload_byte_7;
	uint8_t 	checksum;
	uint8_t		endframe;
};

bool DecodeHeartBeat(struct HEARTBEAT *HeartBeat)
{
	printf("Master HeartBeat: ");
	printf("Total kWh since build : %lukWh, ", (long unsigned int)bswap_32(HeartBeat->totalkWh));
	printf("Voltage Phase A : %dV, ", bswap_16(HeartBeat->phase_a_volts));
	printf("Phase B : %dV, ",         bswap_16(HeartBeat->phase_b_volts));
	printf("Phase C : %dV\r\n\r\n",   bswap_16(HeartBeat->phase_c_volts));
	return(true);
}

bool DecodeFirmware(struct FIRMWARE *FirmwareVer)
{
	printf("Firmware Version %d.%d.%d\r\n\r\n", FirmwareVer->major, FirmwareVer->minor, FirmwareVer->revision);
	return(true);
}

bool DecodeLinkReady(struct LINKREADY *LinkReady)
{
	switch(bswap_16(LinkReady->function)) {
		case RESP_LINK_READY:
			printf("LinkReady 0x%04X: ", bswap_16(LinkReady->function));
			break;
			
		case LINKREADY1:
			printf("LinkReady1 0x%04X: ", bswap_16(LinkReady->function));
			break;
			
		case LINKREADY2:
			printf("LinkReady2 0x%04X: ", bswap_16(LinkReady->function));
			break;
	
		default:
			printf("Unknown function: ");
			break;
	
	}
	printf("Slave ID 0x%04X, ",bswap_16(LinkReady->slaveTWCID));
	printf("Max Charge Rate %0.2fA, ",((float)bswap_16(LinkReady->maxchargerate)/100));
	printf("Sign 0x%02X\r\n\r\n",LinkReady->sign);
	return(true);
}

bool DecodeString(struct STRING *String)
{
	switch(bswap_16(String->function)) {
		case RESP_SERIAL_NUMBER:
			String->string[11] = '\0';
			printf("Serial Number %s\r\n", String->string);
			break;
		case RESP_MODEL_NUMBER:
			printf("Model Number %s\r\n", String->string);
			break;
		case RESP_VIN_FIRST:
			printf("VIN Number:");
		case RESP_VIN_MIDDLE:
		case RESP_VIN_LAST:		
			String->string[10] = '\0';
			printf("%s", &String->string[2]);
			break;
		default:
			printf("Unknown String\r\n");
			break;
	}
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
		case RESP_HEART_BEAT: 	
			DecodeHeartBeat((struct HEARTBEAT *)buffer);
			break;
		case RESP_LINK_READY:
		case LINKREADY1:
		case LINKREADY2:
			DecodeLinkReady((struct LINKREADY *)buffer);
			break;
		case RESP_FIRMWARE_VER:
			DecodeFirmware((struct FIRMWARE *)buffer);
			break;
		case RESP_SERIAL_NUMBER:
		case RESP_MODEL_NUMBER:
		case RESP_VIN_FIRST:
		case RESP_VIN_MIDDLE:
		case RESP_VIN_LAST:
			DecodeString((struct STRING *)buffer);
			break;
		default:
			break;
	}
}

int SendCommand(int fd, uint16_t command)
{
	int nbytes; 
	struct PACKET packet;
	
	packet.startframe = 0xC0;
	packet.function = bswap_16(command);
	packet.TWCID = bswap_16(0x9819);
	packet.payload_byte_0 = 0x00;
	packet.payload_byte_1 = 0x00;
	packet.payload_byte_2 = 0x00;
	packet.payload_byte_3 = 0x00;
	packet.payload_byte_4 = 0x00;
	packet.payload_byte_5 = 0x00;
	packet.payload_byte_6 = 0x00;
	packet.payload_byte_7 = 0x00;
	packet.endframe = 0xC0;	
	packet.checksum = CalculateCheckSum((uint8_t *)&packet, sizeof(packet));
	
	PrintPacket((uint8_t *)&packet, sizeof(packet));
	
	if ((nbytes = write(fd, (uint8_t *)&packet, sizeof(packet))) < 0) {
		perror("Write");
	} else {
		printf("Sent %d bytes\r\n\r\n",nbytes);
	} 
}

int InitCircularBuffer(struct CIRCULAR_BUFFER *cb)
{
	cb->buffer = malloc(cb->max);
	cb->head = 0;
	cb->tail = 0;
}

int PrintCircularBuffer(struct CIRCULAR_BUFFER *cb)
{
	uint16_t i;
	for (i = 0; i <= cb->max; i++)
		printf(" %02X", cb->buffer[i]);
	printf("\r\n");
}

int FreeCircularBuffer(struct CIRCULAR_BUFFER *cb)
{
	free(cb->buffer);
}

int ReadSerialCircularBuffer(int fd, struct CIRCULAR_BUFFER *cb)
{
	int i;
	int nbytes;
	char buffer[32];

	do {
		// Read from serial port... 	
		if ((nbytes = read(fd, &buffer, sizeof(buffer))) < 0) {
			perror("Read");
			return 1;
		} 
		// ...and copy to circular buffer
		if (nbytes != 0) {
			//printf("Read %d bytes ",nbytes);
			for (i = 0; i < nbytes; i++) {
				cb->buffer[cb->head++] = buffer[i];
				//printf(" %02X", (uint8_t)buffer[i]);
				//printf(" %02X %02X\r\n",((cir_buf->head)-1), cir_buf->buffer[cir_buf->head-1]);
				if (cb->head >= cb->max) cb->head = 0;
			}
		}
	} while (nbytes);
}

int ExamineCircularBuffer(struct CIRCULAR_BUFFER *cb)
{
	uint16_t i = cb->tail;
	uint16_t len;
	
	bool StartFrameFound = 0;
	uint16_t StartFrame = 0;
	
	bool EndFrameFound = 0;
	uint16_t EndFrame = 0;
	
	//printf("Examining circular buffer @ %d\r\n", cb->tail);
	
	char buffer[64];

	do {
		// Find start and end byte positions
		if (cb->buffer[i] == 0xC0) { 
			if (!StartFrameFound) {
				StartFrame = i;
				StartFrameFound = true;
			} else {
				EndFrame = i;
				EndFrameFound = true;
				break;
			}
		}
		// Advance position and roll over if required
		if (i++ >= cb->max) i = 0;
	} while (i != cb->head);
	
	//printf("Start frame %03d, End frame %03d\r\n", StartFrame, EndFrame);
	
	if (!(StartFrameFound & EndFrameFound)) {
		//printf("Packet Incomplete, abort.\r\n");
	} else {
		// Calculate packet length
		if (StartFrame < EndFrame) {
			//Frame hasn't rolled over
			len = (EndFrame - StartFrame) + 1;
		} else {
			//Frame has rolled over
			len = ((EndFrame + cb->max) - StartFrame) +1;
		}
		//printf("Start frame %03d, End frame %03d, Length %03d\r\n", StartFrame, EndFrame, len);		

		if (len < 5) {
			printf("Packet too short, ignoring\r\n");
			PrintCircularBuffer(cb);
			// Packet too short. Might be garbage between packets. Advance tail to next 0xC0;
			cb->tail = EndFrame;
			return 0;
		}
		
		// Copy frame to new buffer and pass to functions for parsing
		i = 0;
		cb->tail = StartFrame;
		
		do {
			buffer[i++] = cb->buffer[cb->tail];
			if (++cb->tail >= cb->max) cb->tail = 0;
		} while (i < len);  
	
		cb->tail == EndFrame + 1;
		
		PrintPacket(buffer, i);
		if (VerifyCheckSum(buffer, i)) ProcessPacket(buffer, i);
	}
}

int OpenRS485(const char *devname)
{
	int fd; 
	struct termios options;
		
	if ((fd = open(devname, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
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
	
	return(fd);
}

int main(int argc, char **argv)
{
	int fd; 
	
	if ((fd = OpenRS485("/dev/ttyUSB0")) < 0)
	{
		printf("Cannot open RS-485 port\r\n.");
		return 0;
	}
		
	printf("Port Opened\r\n");
	
	struct CIRCULAR_BUFFER cir_buf;
	cir_buf.max = 256;
	InitCircularBuffer(&cir_buf);
	
	//SendCommand(fd, GET_FIRMWARE_VER);
	//SendCommand(fd, GET_SERIAL_NUMBER);
	//SendCommand(fd, GET_MODEL_NUMBER);
	
	do {
		ReadSerialCircularBuffer(fd, &cir_buf);
		ExamineCircularBuffer(&cir_buf);
		usleep(50000);
		//sleep(1);
		
	} while(1);
	
	FreeCircularBuffer(&cir_buf);

	if (close(fd) < 0) {
		perror("Close");
		return 1;
	}

	return (0);
}