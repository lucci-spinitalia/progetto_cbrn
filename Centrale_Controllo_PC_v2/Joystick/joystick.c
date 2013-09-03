#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include "joystick.h"

int open_joystick(const char *path)
{
  int joystick_fd;
  
  //char joy_name[128];

  joystick_fd = open(path, O_RDONLY); /* read write for force feedback? */
  
  if (joystick_fd < 0)
    return joystick_fd;

  // stamp joystick name
  /*if(ioctl(joystick_fd, JSIOCGNAME(sizeof(joy_name)), joy_name) < 0)
    strncpy(joy_name, "Unknown", sizeof(joy_name));

  printf("Name: %s\n", joy_name);*/

  return joystick_fd;
}

int read_joystick_event(int *joystick_fd, struct js_event *jse)
{
  int bytes;

  bytes = read(*joystick_fd, jse, sizeof(*jse)); 

  return bytes;
}

void close_joystick(int *joystick_fd)
{
  close(*joystick_fd);

  *joystick_fd = -1;
}

int get_joystick_status(int *joystick_fd, struct wwvi_js_event *wjse)
{
  struct js_event jse;
  int bytes_read;

  if (*joystick_fd < 0)
    return -1;

  // memset(wjse, 0, sizeof(*wjse));
  bytes_read = read_joystick_event(joystick_fd, &jse);

  if(bytes_read <= 0)
    return bytes_read;

  //jse.type &= ~JS_EVENT_INIT; /* ignore synthetic events */
  if (jse.type == JS_EVENT_AXIS) {
    switch (jse.number) {
      case 0: 
        wjse->stick_x = jse.value;
        break;

      case 1: 
        wjse->stick_y = jse.value;
        break;

      case 2:
        wjse->stick_z = jse.value;
        break;
	
      default:
        break;
    }
  } 
  else if (jse.type == JS_EVENT_BUTTON) 
  {
    if (jse.number < 10) 
    {
      switch (jse.value) 
      {
        case 0:
        case 1: 
	  wjse->button[jse.number] = jse.value;
          break;
        default:
         break;
      }
    }
  }
  
  return bytes_read;
}

#if 0
/* a little test program */
int main(int argc, char *argv[])
{
  int fd, rc;
  int done = 0;

  struct js_event jse;

  fd = open_joystick();

  if (fd < 0) 
  {
    perror("open_joystick");
    exit(1);
  }


  while (!done) {
    rc = read_joystick_event(&fd, &jse);
    usleep(1000);

    if (rc == 1) 
    {
      printf("Event: time %8u, value %8hd, type: %3u, axis/button: %u\n", 
             jse.time, jse.value, jse.type, jse.number);
    }
  }
}
#endif
