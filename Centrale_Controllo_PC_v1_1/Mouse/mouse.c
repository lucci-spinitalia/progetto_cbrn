#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include "mouse.h"

int open_mouse()
{
  int mouse_fd;
  
  //char joy_name[128];

  mouse_fd = open(MOUSE_DEVNAME, O_RDONLY /*| O_NONBLOCK*/); /* read write for force feedback? */
  
  if (mouse_fd < 0)
    return mouse_fd;

  // stamp joystick name
  /*if(ioctl(joystick_fd, JSIOCGNAME(sizeof(joy_name)), joy_name) < 0)
    strncpy(joy_name, "Unknown", sizeof(joy_name));

    printf("Name: %s\n", joy_name);*/

  return mouse_fd;
}

int read_mouse_event(int *mouse_fd, struct input_event *jse)
{
  int bytes;

  bytes = read(*mouse_fd, jse, sizeof(*jse)); 

  if(bytes == -1)
    return 0;  // nothing to read

  if (bytes == sizeof(*jse))
    return 1;

  return -1;
}

void close_joystick(int *mouse_fd)
{
  close(*mouse_fd);
}

/*  a little test program */
int main(int argc, char *argv[])
{
  int fd, rc;
  int done = 0;
  struct input_event jse;

  fd = open_mouse();

  if (fd < 0) 
  {
    perror("open_mouse");
    exit(1);
  }


  while (!done) {
    rc = read_mouse_event(&fd, &jse);
    usleep(1000);

    if (rc == 1) 
    {
      printf("time %ld.%06ld\ttype %d\tcode%d \tvalue %d\n",
             jse.time.tv_sec, jse.time.tv_usec, jse.type, jse.code, jse.value);
    }
  }

  return 0;
}

