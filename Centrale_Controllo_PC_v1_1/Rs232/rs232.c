#include <sys/types.h>
#include <linux/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include <ctype.h>

#include "rs232.h"

#define BAUDRATE B115200
#define MODEMDEVICE "/dev/ttyO2"
#define _POSIX_SOURCE 1
#define FALSE 0
#define TRUE 1

volatile int STOP=FALSE;


int com_open(char *device_name, __u32 rate, char parity,
             int data_bits, int stop_bits)
{
  int fd;
  int local_rate = 0;
  int local_databits = 0;
  int local_stopbits = 0;
  int local_parity = 0;
  char upper_parity;
  struct termios oldtio, newtio;

  fd = open(device_name, O_RDWR | O_NOCTTY);

  if(fd < 0)
  {
    perror(MODEMDEVICE);
    exit(-1);
  }

  // Check fo valid values
  upper_parity = toupper(parity);

  if(((data_bits == 5) || (data_bits == 6) || (data_bits == 7) || (data_bits == 8)) &&
     ((stop_bits == 2) || (stop_bits == 1)) &&
     ((upper_parity == 'N') || (upper_parity == 'O') || (upper_parity == 'E')) &&
     ((rate == 50) || (rate == 75) || (rate == 110) || (rate == 134) || (rate == 150) ||
      (rate == 200) || (rate == 300) || (rate == 600) || (rate == 1200) || (rate == 38400) ||
      (rate == 57600) || (rate == 115200)))
  {
    switch(rate)
    {
      case 50: local_rate = B50; break;
      case 75: local_rate = B75; break;
      case 110: local_rate = B110; break;
      case 134: local_rate = B134; break;
      case 150: local_rate = B150; break;
      case 200: local_rate = B200; break;
      case 300: local_rate = B300; break;
      case 600: local_rate = B600; break;
      case 1200: local_rate = B1200; break;
      case 1800: local_rate = B1800; break;
      case 2400: local_rate = B2400; break;
      case 4800: local_rate = B4800; break;
      case 9600: local_rate = B9600; break;
      case 19200: local_rate = B19200; break;
      case 38400: local_rate = B38400; break;
      case 57600: local_rate = B57600; break;
      case 115200: local_rate = B115200; break;
    }

    switch(data_bits)
    {
      case 5: local_databits = CS5; break;
      case 6: local_databits = CS6; break;
      case 7: local_databits = CS7; break;
      case 8: local_databits = CS8; break;
    }

    if(stop_bits == 2)
      local_stopbits = CSTOPB;
    else
      local_stopbits = 0;

    switch(upper_parity)
    {
      case 'E': local_parity = PARENB; break;
      case 'O': local_parity |= PARODD; break;
    } 


    tcgetattr(fd, &oldtio); //save current port settings
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = local_rate | CRTSCTS | local_databits | local_stopbits | local_parity | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; //inter-character timer unused
    newtio.c_cc[VMIN] = 1; //blocking read until 5 chars received

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    return fd;
  }
  else
  {
    close(fd);
    return -1;
  }
}

/*
int main()
{
  fd_set rset;
  int status = 0;

  int rs232_device, res;
  char buf[255];

  rs232_device = com_open(MODEMDEVICE, 115200, 'N', 8, 1);

  while(STOP == FALSE)
  {
    FD_ZERO(&rset);

    if(rs232_device > 0)
      FD_SET(rs232_device, &rset);

    status = select(rs232_device + 1, &rset, NULL, NULL, NULL);

    if(status > 0)
    {
      if(FD_ISSET(rs232_device, &rset))
      { 
        res = read(rs232_device, buf, 255);
        buf[res] = 0;
        printf(":%s:%d\n", buf, res);
        write(rs232_device, buf, res);
  
        if(buf[0] == 'z')
          STOP = TRUE;
      }
    }
  }

//  tcsetattr(rs232_device, TCSANOW, &oldtio); 

  return 0;
}*/
