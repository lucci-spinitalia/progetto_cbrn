#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>

int kbport = -1;

int posix_kbhit(void)
{
  int rtnval = 0;
  
  fd_set rset;
  struct timeval tvptr;
  struct termios tty;

  int status = 0;

  if(kbport == -1)
  {
    // Open the controlling terminal for read-only access
    kbport = open(ctermid(NULL), O_RDONLY);

    if(kbport > -1)
    {
      // Get the terminal attributes
      if(tcgetattr(kbport, &tty) < 0)
      {
        fprintf(stderr, "\ERROR: unable to get tty attributes for %s\n", ctermid(NULL));
        exit(-1);
      }
      else
      {
        tty.c_iflag = 0x0000;  // we don't need any input flags
        tty.c_oflag = ONLCR | OPOST;
        tty.c_cflag = CREAD | CSIZE;
        tty.c_lflag = IEXTEN; // Note - noncanonical mode, since no ICANON

        tty.c_cc[VERASE] = 0X7f; /* Since we're in noncanonical mode, we ignore
                                    backspace, and its value *should* be 127;
                                    to enable me to handle it, I make *sure* it's
                                    127, and then have e special case to map 127
                                    to 8 (^H) inside posix_getkey() */

        if(tcsetattr(kbport, TCSANOW, &tty) < 0)
        {
          fprintf(stderr, "\nERROR: unable to set tty attributes for %s\n", ctermid(NULL));
        }

        // set up our "return immediately" structure for select
        tvptr.tv_sec = 0;
        tvptr.tv_usec = 0;
      }
    }

    //Initialize the read set to zero
    FD_ZERO(&rset);

    //turn on the read set
    FD_SET(kbport, &rset);

    status = select(kbport + 1, &rset, NULL, NULL, NULL);

    if(status == -1) // Error
    {
      fprintf(stderr, "\nERROR: kbhit returned -1\n");
      exit(-1);
    }
    else if(status > 0)
    {
      if(FD_ISSET(kbport, &rset))
        rtnval = 1;
    }
  }
  
  return rtnval;
}

char posix_getkey(char *dst)
{
  char ch = 0;
  char rtnval = 0;

  if(kbport != -1)
  {
    read(kbport, &ch, 1);
    
    *dst = ch;
    rtnval = 0;
  }

  return rtnval;
}


int rs232_init(char *device_name, struct termios *config)
{
  int fd;

  memset(config, 0, sizeof(*config));

  fd = open(device_name, O_RDWR | O_NOCTTY);

  if(fd == -1)
    return -1;

  if(!isatty(fd))
    return -1;

  if(tcgetattr(fd, config) < 0)
    return -1;

  cfmakeraw(config);

  config->c_cc[VMIN] = 1;
  config->c_cc[VTIME] = 0;

  if(tcsetattr(fd, TCSAFLUSH, config) < 0)
    return -1;

  return fd;
}

struct termios *get_termsettings(char *devname)
{
  int port = 0;
  struct termios *rtnval = NULL;

  if(devname != (char *) NULL)
  {
    port = open(devname, O_RDONLY);

    if(port > -1)
    {
      rtnval = (struct termios *) malloc(sizeof(struct termios));

      if(rtnval != (struct termios *) NULL)
      {
        if(tcgetattr(port, rtnval) == -1)
        {
          free(rtnval);
          rtnval = (struct termios *) NULL;
        }
      }

      close(port);
    }
  }

  return(rtnval);
}


int set_termsettings(char *devname, struct termios *settings)
{
  int port = 0;
  int rtnval = 0;

  if(devname != (char *) NULL)
  {
    port = open(devname, O_RDWR);

    if(port > -1)
    {
      if(tcsetattr(port, TCSANOW, settings) != -1)
        rtnval = 1;

      close(port);
    }
  }

  return rtnval;
}

int com_open(char *devicename, int rate, char parity,
             int databits, int stopbits, int options)
{
  int rtnval = 0;
  struct termios t;

  int local_databits = 0;
  int local_stopbits = 0;
  int local_parity = 0;
  int local_rate = 0;
  char upper_parity = 0;

  // Check for valid values
  upper_parity = toupper(parity);
  
  if(((databits == 5) || (databits == 6) || (databits == 7) || (databits == 8)) &&
     ((stopbits == 2) || (stopbits == 1)) &&
     ((upper_parity == 'N') || (upper_parity == 'O') || (upper_parity == 'E')) &&
     ((rate == 50) || (rate == 75) || (rate == 110) || (rate == 134) || (rate == 150) ||
      (rate == 200) || (rate == 300) || (rate == 600) || (rate == 1200) || (rate == 38400) ||
      (rate == 57600) || (rate == 115200)))
  {
    // open the com port for read/write access
    rtnval = open(devicename, O_RDWR | O_NOCTTY);

    if(rtnval != -1)
    {
      //Set th parity, databits, and stop bits
      switch(databits)
      {
        case 5: local_databits = CS5; break;
        case 6: local_databits = CS6; break;
        case 7: local_databits = CS7; break;
        case 8: local_databits = CS8; break;
      }

      if(stopbits == 2)
        local_stopbits = CSTOPB;
      else
        local_stopbits = 0;

      switch(upper_parity)
      {
        case 'E': local_parity = PARENB; break;
        case 'O': local_parity |= PARODD; break;
      }
    }
  
    bzero(&t, sizeof(t));
    
    t.c_iflag = IGNPAR;
    t.c_cflag = CLOCAL | CREAD | B115200 | CS8 | CRTSCTS;
    t.c_oflag = 0;
    t.c_lflag = 0;

    t.c_cc[VTIME] = 0;
    t.c_cc[VMIN] = 1;

    tcflush(rtnval, TCIFLUSH);
    tcsetattr(rtnval, TCSANOW, &t);

    // Set the data rate
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
#ifndef __OS_IRIX
      case 57600: local_rate = B57600; break;
      case 115200: local_rate = B115200; break;
#endif
    }
/*
    if((cfsetispeed(&t, local_rate) != -1) && (cfsetospeed(&t, local_rate) != -1))
    {
      if(tcsetattr(rtnval, TCSANOW, &t) == -1)
        rtnval= -1;
    }
    else
      rtnval = -1;*/
  }
  else
    rtnval = -1;

  return rtnval;
}

int com_dataready(int port)
{
  struct timeval tvptr;
  fd_set rset;
  int status = -1;
  int rtnval = 0;

  if(port != -1)
  {
    //Zero out the seconds and microseconds so we don't block
    tvptr.tv_sec = 0;
    tvptr.tv_usec = 0;

    //Initialize the read set to zero
    FD_ZERO(&rset);

    //Turn on the read set
    FD_SET(port, &rset);

    printf("\nWaiting for characters \n");
    fflush(stdout);
    status = select(port + 1, &rset, NULL, NULL, NULL);

    printf("\nCharacter received\n");
    fflush(stdout);

    if(status == -1) //Error
    {
      printf("\nERROR: dataready() returned -1\n");
      rtnval = 0;
    }
    else if(status > 0)
    {
      if(FD_ISSET(port, &rset))
        rtnval = 1;
    }
  }

  return rtnval;
}



char com_read(int port, char *dst)
{
  if(port != -1)
  {
    printf("Waiting for char\n");
    return read(port, dst, 1);
  }
  else
    return -1;
}

char com_write(int port, char src)
{
  if(port != -1)
    return(write(port, &src, 1) == 1);
  else
    return 0;
}

int main()
{
  char done = 0;
  char funckey = 0;
  char key = 0;
  
  int rs232_device = 0;
  struct termios rs232_config;
  struct termios *config = NULL;
  char buf[255];
  char bytes_read = 0;

  //rs232_device = rs232_init("/dev/ttyO2", &rs232_config);
  rs232_device = com_open("/dev/ttyO2", 115200, 'N', 8, 1, 0);
  config = get_termsettings(ctermid(NULL));

  if(rs232_device != -1)
  {
    while(!done)
    {
/*      if(com_dataready(rs232_device))
      {
        printf("Char received");
        fflush(stdout);

        rs232_return = com_read(rs232_device, &inchar);

        if(rs232_return > 0)
        {
          if(isprint(inchar))
            printf("%c", inchar);
          else
            printf("<%03d>",inchar);
        
          fflush(stdout);
        }
        else if(rs232_return == -1)
          perror("com_read");
      }

      if(posix_kbhit())
      {
        funckey = posix_getkey(&key);
        done = ((key == 27) && (!funckey));
        if(!done)
          com_write(rs232_device, key);
      }*/
  
      bytes_read = read(rs232_device, &buf, 255);

      if(bytes_read > 0)
        printf("%s", buf);
  
    }
      
    //set_termsettings(ctermid(NULL), config);
  }

  return 0;
}
