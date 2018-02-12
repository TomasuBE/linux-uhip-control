/*
* File     : uart_transport.c
* Created  : December 2017
* Updated  : 04-12-2017
* Author   : Thomas Brijs
* Synopsis : HostCPU Transport (physical) Interface PC UART implementation
*
* This is an implementation of the UART peripheral interface on the Host CPU
*
* Copyright 2017, 
*/

#include "hostcpu_transport.h"

#include <errno.h>
#include <fcntl.h> 
#include <signal.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "uhip_structures.h"

#define RX_BUF_SIZE 1024
#define FALSE		0
#define TRUE		1
#define BAUDRATE	B115200
#define PARITY		0

//#define	DEBUG_INFO	1	//debug messages

static uint32_t baud = BAUDRATE;
static uint8_t rx_in_buffer[RX_BUF_SIZE];
static size_t rx_read_idx = 0;
static size_t total_rx = 0;

int fd = 0;

static void cleanup(void);
static uint32_t read_data(void);

char *portname = "/dev/ttyUSB0";


void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                fprintf( stderr, "error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 1; //5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                fprintf( stderr,"error %d setting term attributes", errno);
}


static uint32_t read_data()
{	
	uint32_t num_data = 0;
	
	num_data = read(fd, rx_in_buffer, sizeof(rx_in_buffer));

	if (num_data < 0)
	{
		AUD_PRINTF("Unable to read from serial port\n");
	}

	//fprintf( stderr,"read %i\n", errno);
#ifdef DEBUG_INFO
	if(num_data)
		fprintf( stdout,"[%i]", num_data );
#endif
	return num_data;
}

static void cleanup()
{
	if (fd)
		close(fd);
}

void override_default_com_port_settings(char *new_portname, uint32_t new_baud)
{
	if (new_portname) {
		portname = new_portname;
	}
	if (new_baud) {
		baud = new_baud;
	}
}

aud_bool_t hostcpu_transport_init(void)
{
	struct termios tty;

	bzero(&tty, sizeof(tty));
	
	//fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	/* Open File Descriptor */
	fd = open( portname, O_RDWR| O_NONBLOCK | O_NDELAY | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		AUD_PRINTF("Unable to open COM port\n");
		return AUD_FALSE;
	}
	
	memset (&tty, 0, sizeof tty);
	
	if (tcgetattr (fd, &tty) != 0)
	{
			fprintf( stderr, "error %d from tcgetattr", errno);
			return -1;
	}

	cfsetospeed (&tty, BAUDRATE);
	cfsetispeed (&tty, BAUDRATE);
	
	tty.c_cflag     &=  ~PARENB;        // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;
	tty.c_cflag     &=  ~CRTSCTS;       // no flow control
	tty.c_lflag     =   0;          // no signaling chars, no echo, no canonical processing
	tty.c_oflag     =   0;                  // no remapping, no delays
	tty.c_cc[VMIN]      =   0;                  // read doesn't block
	tty.c_cc[VTIME]     =   0;                  // 0.5 seconds read timeout

	tty.c_iflag		|= IGNBRK;
	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
	tty.c_iflag     &=  ~(IXON | IXOFF | IXANY);// turn off s/w flow ctrl
	tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	tty.c_oflag     &=  ~OPOST;              // make raw

	if (tcsetattr(fd,TCSANOW,&tty) != 0)
	{
			fprintf( stderr, "error %d from tcsetattr", errno);
			return -1;
	}
	
	return AUD_TRUE;
}

/**
* Write bytes to the transport interface
* @param buffer [in] Pointer to the buffer to write
* @param num_bytes [in] The number of bytes to write
* @return the number of bytes written
*/
size_t hostcpu_transport_write(uint8_t const * buffer, size_t num_bytes)
{
	uint32_t i;
	uint32_t chunks = num_bytes / UHIP_CHUNK_SIZE;
	uint32_t num_bytes_written;

	if (!num_bytes) {
		return 0;
	}
	
#ifdef DEBUG_INFO
	fprintf(stdout, "DEBUG: writing %u bytes: %s\n",num_bytes, buffer);
#endif

	if (chunks)
	{
		for (i = 0; i < chunks; ++i)
		{
#ifdef DEBUG_INFO
			fprintf(stdout, "DEBUG: writing chunk %i of %i\n", i+1, chunks);
#endif
		
			num_bytes_written = write(fd, &buffer[i * UHIP_CHUNK_SIZE], UHIP_CHUNK_SIZE);
			
			if (num_bytes_written < 0)
			{
				AUD_PRINTF("Unable to write data\n");
				return 0;
			}
			if (num_bytes_written == 0)
			{
				AUD_PRINTF("Did not write any data to the serial port\n");
				return 0;
			}
		}
	}
	else
	{
		num_bytes_written = write(fd, &buffer, num_bytes);
		
#ifdef DEBUG_INFO
		fprintf(stdout, "DEBUG: written %i bytes of %i\n", num_bytes_written, num_bytes);
#endif
		
		if (num_bytes_written < 0)
		{
			AUD_PRINTF("Unable to write data\n");
			return 0;
		}
		if (num_bytes_written == 0)
		{
			AUD_PRINTF("Did not write any data to the serial port\n");
			return 0;
		}
	}


	return num_bytes;
}

/**
* Read bytes from the transport interface
* @param buffer [in] Pointer to the buffer to read into
* @param max_bytes_to_read [in] The maximum number of bytes to read
* @return the number of bytes read
*/
size_t hostcpu_transport_read(uint8_t* buffer, size_t max_bytes_to_read)
{
	uint8_t pbuf = &buffer;
	
	if (total_rx == 0)
	{
		total_rx = read_data();
		
		if (total_rx == 0) {
			return 0;
		}
	}
#ifdef DEBUG_INFO
	fprintf( stdout,"DEBUG: recvd %i\n", total_rx);
#endif
	size_t data_size;

	if (total_rx >= max_bytes_to_read) {
		data_size = max_bytes_to_read;
	}
	else {
		data_size = total_rx;
	}

	memcpy(buffer, &rx_in_buffer[rx_read_idx], data_size);

	rx_read_idx = (rx_read_idx + data_size);
	total_rx -= data_size;

	if (total_rx == 0) {
		rx_read_idx = 0;
	}

	//bits uitlijnen ? pointer opschuiven tot na 1e byte
	pbuf += sizeof(uint8_t);

#ifdef DEBUG_INFO
		fprintf( stdout,"DEBUG: read %i bytes containing %s @ %p\n", data_size, buffer, pbuf);
#endif
		

	return data_size;
}
