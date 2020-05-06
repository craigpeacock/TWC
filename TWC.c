/*
Tesla Wall Connector (TWC) Example
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
#include <time.h>

// Commands from https://teslamotorsclub.com/tmc/posts/3225600/
// Commands with reponses (0xFB)
#define GET_SERIAL_NUMBER	0xFB19
#define GET_MODEL_NUMBER	0xFB1A
#define GET_FIRMWARE_VER 	0xFB1B
#define GET_PLUG_STATE		0xFBB4

#define MASTER_HEATBEAT		0xFBE0
#define LINKREADY2		0xFBE2

#define GET_VIN_FIRST		0xFBEE
#define GET_VIN_MIDDLE		0xFBEF
#define GET_VIN_LAST		0xFBF1

// Commands without responses (0xFC)
#define START_CHARGING		0xFCB1
#define STOP_CHARGING		0xFCB2
#define LINKREADY1		0xFCE1

// Responses (0xFD)
#define RESP_SERIAL_NUMBER	0xFD19
#define RESP_MODEL_NUMBER	0xFD1A
#define RESP_FIRMWARE_VER	0xFD1B
#define RESP_PLUG_STATE		0xFDB4

#define SLAVE_HEARTBEAT		0xFDE0

#define RESP_LINK_READY		0xFDE2		// Sent by slave on reset
#define RESP_PWR_STATUS		0xFDEB		// Sent by master on reset
#define RESP_VIN_FIRST		0xFDEE
#define RESP_VIN_MIDDLE		0xFDEF
#define RESP_VIN_LAST		0xFDF1

#pragma pack(1)

struct CIRCULAR_BUFFER {
	uint8_t 	* buffer;
	uint16_t 	head;
	uint16_t 	tail;
	uint16_t 	max;
	bool 		full;
};

// PACKET: Basic packet structure. Used to send GET_* commands.
// Each frame begins with a 0xC0 byte and ends with a 0xC0 byte.

struct PACKET {
	uint8_t		startframe;		// 0xC0
	uint16_t	function;		// Function (command)
	uint16_t	src_TWCID;		// Address of source TWC
	uint16_t	dest_TWCID;		// Address of destination TWC
	uint8_t		payload_byte[6];
	uint8_t 	checksum;		// Checksum
	uint8_t		endframe;		// 0xC0
};

// POWERSTATUS: Sent periodically from a master (approx every 2 seconds) with
// power statistics such as total power delivered by the Tesla Wall Connector
// since construction (totalkWh), line voltages and currents.

struct POWERSTATUS {
	uint8_t		startframe;
	uint16_t	function;		// Should be 0xFDEB
	uint16_t	src_TWCID;		// Address of source TWC
	uint32_t	totalkWh;		// Total kWh since construction
	uint16_t	line1_volts;		// Line Voltage
	uint16_t	line2_volts;
	uint16_t	line3_volts;
	uint8_t		line1_current;		// Line Current * 2
	uint8_t		line2_current;
	uint8_t		line3_current;
	uint8_t		dummy_bytes[2];
	uint8_t 	checksum;
	uint8_t		endframe;
};

// M_HEARTBEAT: Heartbeat packet sent by master. Used to tell slave to adjust
// charging current. Packet must be sent regularly to prevent TWC entering a
// fault (Red LED) mode. If there is no need to make any adjustments to
// allocated current, you can send a NOP. Each Heartbeat will be acknowledged
// with a slave heartbeat (S_HEARTBEAT)

struct M_HEARTBEAT {
	uint8_t		startframe;
	uint16_t	function;		// Should be 0xFBE0
	uint16_t	src_TWCID;		// Our address (Master)
	uint16_t	dest_TWCID;		// Address of target slave
	uint8_t		command;		// 0x00 NOP
						// 0x02 Error
						// 0x05 Limit power to <max current> (Protocol 1)
						// 0x06 Increase charge current by 2 amps
						// 0x07 Decrease charge current by 2 amps
						// 0x08 Master ack slave stopped charging
						// 0x09 Limit power to <max current> (Protocol 2)
	uint16_t	max_current;		// Maximum current slave can draw
	uint8_t		master_plug_inserted; 	// 0x01 if master has plug inserted
	uint8_t		dummy_bytes[3];
	uint8_t 	checksum;
	uint8_t		endframe;
};

// S_HEARTBEAT: Heartbeat packet sent by slave. Used to acknowledge master
// heartbeat and report maximum allocated and actual charging current.

struct S_HEARTBEAT {
	uint8_t		startframe;
	uint16_t	function;		// Should be 0xFDE0
	uint16_t	src_TWCID;		// Address of Slave
	uint16_t	dest_TWCID;		// Address of Master
	uint8_t		status;			// 0x00 Ready
						// 0x01	Charging
						// 0x02 Error
						// 0x03 Plugged in, do not charge
						// 0x04 Plugged in, ready to charge or charge scheduled
						// 0x05 Busy?
						// 0x06 +Ack to Increase charge current by 2 amps
						// 0x07 +Ack to Decrease charge current by 2 amps
						// 0x08 Starting to charge?
						// 0x09 +Ack to Limit power to <max current> (Protocol 2)
	uint16_t	max_current;		// Maximum current slave can draw. Only valid when slave
						// is connected to vehicle, otherwise reports zero.
	uint16_t	actual_current;		// Actual current being drawn by car connected to slave
	uint8_t		dummy_bytes[2];
	uint8_t 	checksum;
	uint8_t		endframe;
};

// FIRMWARE: Response to GET_FIRMWARE_VER command containing firmware version
// of the TWC.

struct FIRMWARE {
	uint8_t		startframe;
	uint16_t	function;		// Should be 0xFD1B
	uint8_t		major;			// Major version
	uint8_t		minor;			// Minor version
	uint8_t		revision;		// Revision
	uint8_t		dummy_bytes[8];
	uint8_t		checksum;
	uint8_t		endframe;
};

// STRING: Response to commands requesting a string. Strings are not normally
// zero terminated.

struct STRING {
	uint8_t		startframe;
	uint16_t	function;		// Function code
	uint8_t		string[11];		// ASCII string
	uint8_t		checksum;		// Checksum
	uint8_t		endframe;
};

// PLUGSTATE: Response to GET_PLUG_STATE command containing plug status of TWC.

struct PLUGSTATE {
	uint8_t		startframe;
	uint16_t	function;		// Should be RESP_PLUG_STATE 0xFDB4
	uint16_t	src_TWCID;		// Address of source TWC
	uint8_t		plug_state;		// 0x00 Unplugged
						// 0x01 Charging
						// 0x02 <unknown>
						// 0x03 Plugged in, but not charging
	uint8_t		dummy_bytes[10];
	uint8_t 	checksum;
	uint8_t		endframe;
};

// LINKREADY: Sent periodically from a slave. A master should respond with a M_HEARTBEAT.
// Used to advertise the presence of a slave and its TWCID. Also advertises the maximum
// charge rate the hardware is capable of.

struct LINKREADY {
	uint8_t		startframe;
	uint16_t	function;		// 0xFCE1 LinkReady1 or 0xFCE2 LinkReady 2
	uint16_t	src_TWCID;		// Address of source TWC
	uint8_t		sign;			// Random 'sign', not sure its purpose
	uint16_t	max_charge_rate;	// Maximum charge rate in amps * 100
	uint8_t		dummy_bytes[8];
	uint8_t 	checksum;
	uint8_t		endframe;
};

int fd;
uint8_t VIN[22];
uint16_t Slave_TWCID;
uint16_t Master_TWCID = 0xA5A5;

int SendMasterHeartbeat(int fd, uint16_t Slave_TWCID, uint16_t max_current);
int SendCommand(int fd, uint16_t command, uint16_t src_id, uint16_t dest_id);

bool DecodePowerStatus(struct POWERSTATUS *PowerStatus)
{
	printf("Power statistics from TWC%04X: ", bswap_16(PowerStatus->src_TWCID));
	printf("Total kWh: %lukWh, ", (long unsigned int)bswap_32(PowerStatus->totalkWh));
	printf("L1: %dV %.0fA, ",	bswap_16(PowerStatus->line1_volts), (float)PowerStatus->line1_current/2);
	printf("L2: %dV %.0fA, ",	bswap_16(PowerStatus->line2_volts), (float)PowerStatus->line2_current/2);
	printf("L3: %dV %.0fA\r\n\r\n",	bswap_16(PowerStatus->line3_volts), (float)PowerStatus->line3_current/2);
	return(true);
}

bool DecodeFirmware(struct FIRMWARE *FirmwareVer)
{
	printf("Firmware Version %d.%d.%d\r\n\r\n", FirmwareVer->major, FirmwareVer->minor, FirmwareVer->revision);
	return(true);
}

bool DecodePlugState(struct PLUGSTATE *PlugState)
{
	printf("Plug state from TWC%04X: ", bswap_16(PlugState->src_TWCID));
	switch (PlugState->plug_state) {
		case 0x00:
			printf("Unplugged\r\n\r\n");
			break;
		case 0x01:
			printf("Charging\r\n\r\n");
			break;
		case 0x02:
			printf("??\r\n\r\n");
			break;
		case 0x03:
			printf("Plugged in, but not charging\r\n\r\n");
			break;
		default:
			printf("Unknown status %d\r\n\r\n",PlugState->plug_state);
			break;
	}

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
	printf("Slave ID 0x%04X, ",bswap_16(LinkReady->src_TWCID));
	printf("Max Charge Rate %0.2fA, ",((float)bswap_16(LinkReady->max_charge_rate)/100));
	printf("Sign 0x%02X\r\n\r\n",LinkReady->sign);
	return(true);
}


bool DecodeMasterHeartbeat(struct M_HEARTBEAT * Heartbeat)
{
	printf("Master Heartbeat: Src 0x%04X, Dest 0x%04X ", bswap_16(Heartbeat->src_TWCID), bswap_16(Heartbeat->dest_TWCID));
	switch (Heartbeat->command) {
		case 0x00:
			printf("NOP");
			break;
		case 0x02:
			printf("Error");
			break;
		case 0x05:
			printf("Limit current to %.02f", (float)bswap_16(Heartbeat->max_current) /100);
			break;
		case 0x06:
			printf("Increase charge current by 2 amps");
			break;
		case 0x07:
			printf("Decrease charge current by 2 amps");
			break;
		case 0x08:
			printf("Ack Slave has stopped charging");
			break;
		case 0x09:
			printf("Limit current to %.02f", (float)bswap_16(Heartbeat->max_current) /100);
			break;
		default:
			printf("Unknown status (%d)", Heartbeat->command);
	}
	printf("\r\n");
}

bool DecodeSlaveHeartbeat(struct S_HEARTBEAT *Heartbeat)
{
	printf("Slave Heartbeat: Src 0x%04X, Dest 0x%04X ", bswap_16(Heartbeat->src_TWCID), bswap_16(Heartbeat->dest_TWCID));
	switch (Heartbeat->status) {
		case 0x00:
			printf("Ready");
			break;
		case 0x01:
			printf("Charging");
			break;
		case 0x02:
			printf("Error");
			break;
		case 0x03:
			printf("Plugged in, do not charge");
			break;
		case 0x04:
			printf("Plugged in, ready to charge or charge scheduled");
			break;
		case 0x05:
			printf("Busy?");
			break;
		case 0x06:
			printf("+Ack to Increase charge current by 2 amps");
			break;
		case 0x07:
			printf("+Ack to Decrease charge current by 2 amps");
			break;
		case 0x08:
			printf("Starting to charge?");
			break;
		case 0x09:
			printf("+Ack to Limit power to <max current>");
			break;
		default:
			printf("Unknown status (%d)", Heartbeat->status);
	}
	printf("\r\n");
	printf("Slave Maximum Current %.02f, Actual Current %.02f\r\n\r\n",
			(float)bswap_16(Heartbeat->max_current) /100,
			(float)bswap_16(Heartbeat->actual_current) / 100);
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
			printf("Obtained first 7 bytes of VIN\r\n\r\n");
			strncpy(&VIN[0], &String->string[2],7);
			SendCommand(fd, GET_VIN_MIDDLE, 0x0000, 0x9819);
			break;
		case RESP_VIN_MIDDLE:
			printf("Obtained middle 7 bytes of VIN\r\n\r\n");
			strncpy(&VIN[7], &String->string[2],7);
			SendCommand(fd, GET_VIN_LAST, 0x0000, 0x9819);
			break;
		case RESP_VIN_LAST:
			strncpy(&VIN[14], &String->string[2],7);
			VIN[20] = '\0';
			printf("VIN: %s\r\n\r\n", VIN);
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
		case RESP_PWR_STATUS:
			DecodePowerStatus((struct POWERSTATUS *)buffer);
			break;
		case RESP_LINK_READY:
			DecodeLinkReady((struct LINKREADY *)buffer);
			// Slave has advertised its address. Copy to Slave_TWCID and use
			// this for subsequent communication.
			Slave_TWCID = bswap_16(packet->src_TWCID);
			break;
		case LINKREADY1:
		case LINKREADY2:
			DecodeLinkReady((struct LINKREADY *)buffer);
			break;
		case SLAVE_HEARTBEAT:
			DecodeSlaveHeartbeat((struct S_HEARTBEAT *)buffer);
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
		case RESP_PLUG_STATE:
			DecodePlugState((struct PLUGSTATE *)buffer);
		default:
			break;
	}
}

int SendCommand(int fd, uint16_t command, uint16_t src_id, uint16_t dest_id)
{
	int nbytes;
	struct PACKET packet;

	packet.startframe = 0xC0;
	packet.function = bswap_16(command);
	packet.src_TWCID = bswap_16(src_id);
	packet.dest_TWCID = bswap_16(dest_id);
	packet.payload_byte[0] = 0x00;
	packet.payload_byte[1] = 0x00;
	packet.payload_byte[2] = 0x00;
	packet.payload_byte[3] = 0x00;
	packet.payload_byte[4] = 0x00;
	packet.payload_byte[5] = 0x00;
	packet.endframe = 0xC0;
	packet.checksum = CalculateCheckSum((uint8_t *)&packet, sizeof(packet));

	PrintPacket((uint8_t *)&packet, sizeof(packet));

	if ((nbytes = write(fd, (uint8_t *)&packet, sizeof(packet))) < 0) {
		perror("Write");
	} else {
		printf("Sent %d bytes\r\n\r\n",nbytes);
	}
}

int SendMasterHeartbeat(int fd, uint16_t Slave_TWCID, uint16_t max_current)
{
	int nbytes;
	struct M_HEARTBEAT heartbeat;

	heartbeat.startframe = 0xC0;
	heartbeat.function = bswap_16(MASTER_HEATBEAT);
	heartbeat.src_TWCID = bswap_16(Master_TWCID);
	heartbeat.dest_TWCID = bswap_16(Slave_TWCID);
	heartbeat.command = 0x09;
	heartbeat.max_current = bswap_16(max_current);
	heartbeat.master_plug_inserted = 0x00; 	// 0x01 if master has plug inserted
	heartbeat.dummy_bytes[0] = 0x00;
	heartbeat.dummy_bytes[1] = 0x00;
	heartbeat.dummy_bytes[2] = 0x00;
	heartbeat.endframe = 0xC0;
	heartbeat.checksum = CalculateCheckSum((uint8_t *)&heartbeat, sizeof(heartbeat));

	PrintPacket((uint8_t *)&heartbeat, sizeof(heartbeat));
	DecodeMasterHeartbeat(&heartbeat);

	if ((nbytes = write(fd, (uint8_t *)&heartbeat, sizeof(heartbeat))) < 0) {
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
			buffer[i] = cb->buffer[cb->tail];

			// Check for escape bytes: TWC uses same scheme than SLIP
			// https://en.wikipedia.org/wiki/Serial_Line_Internet_Protocol
			if ((uint8_t)buffer[i] == 0xDB) {
				// Reduce packet size
				len--;
				// Advance pointer to get next byte and check for rollover
				if (++cb->tail >= cb->max) cb->tail = 0;
				if (cb->buffer[cb->tail] == 0xDD) buffer[i] = 0xDB;
				if (cb->buffer[cb->tail] == 0xDC) buffer[i] = 0xC0;
			}

			i++;
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

	options.c_lflag &= ~ISIG;

	options.c_lflag &= ~ICANON;	// Disable canonical mode
	options.c_cc[VMIN]=0;		// Minimum number of characters in noncanonical mode
	options.c_cc[VTIME]=0;		// timeout in deciseconds

	// Set port attributes
	tcsetattr(fd, TCSAFLUSH, &options);

	return(fd);
}

int main(int argc, char **argv)
{
	struct timespec ts;
	time_t oldtime;

	if ((fd = OpenRS485("/dev/ttyUSB0")) < 0)
	{
		printf("Cannot open RS-485 port\r\n.");
		return 0;
	}

	printf("Port Opened\r\n");

	// Initially set Slave_TWCID to zero. When we receive LinkReady packet, Slave_TWCID will be populated.
	Slave_TWCID = 0x0000;

	struct CIRCULAR_BUFFER cir_buf;
	cir_buf.max = 256;
	InitCircularBuffer(&cir_buf);

	//SendCommand(fd, GET_FIRMWARE_VER, 0x9819, 0x0000);
	//SendCommand(fd, GET_SERIAL_NUMBER, 0x9819, 0x0000);
	//SendCommand(fd, GET_MODEL_NUMBER, 0x9819, 0x0000);
	//SendCommand(fd, GET_VIN_FIRST, 0x0000, 0x9819);
	//SendCommand(fd, START_CHARGING, 0xAA55, 0x9819);
	//SendCommand(fd, STOP_CHARGING, 0xAA55, 0x9819);
	//SendCommand(fd, GET_PLUG_STATE, 0x0000, 0x0000);

	clock_gettime(CLOCK_REALTIME, &ts);
	oldtime = ts.tv_sec;

	do {
		ReadSerialCircularBuffer(fd, &cir_buf);
		ExamineCircularBuffer(&cir_buf);
		usleep(10000);

		clock_gettime(CLOCK_REALTIME, &ts);
		if (ts.tv_sec != oldtime) {

			// Respond with master heartbeat
			if (Slave_TWCID) SendMasterHeartbeat(fd, Slave_TWCID, 700);

			oldtime = ts.tv_sec;
		}

	} while(1);

	FreeCircularBuffer(&cir_buf);

	if (close(fd) < 0) {
		perror("Close");
		return 1;
	}

	return (0);
}
