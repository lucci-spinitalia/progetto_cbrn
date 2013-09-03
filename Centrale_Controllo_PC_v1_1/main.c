#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/time.h>
#include <time.h>
#include <errno.h>

#include <libudev.h>

#include <locale.h>

#include <string.h>

#include <linux/joystick.h>

#include <sys/mount.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <netinet/in.h>

#include "./Socket_can/socket_can_interface.h"
#include "./Segway/segway.h"
#include "./Joystick/joystick.h"
#include "./Rs232/rs232.h"
 
#define JOY_LOCAL_NAME "/dev/input/js0"
#define JOY_MAX_VALUE 32767
#define TIMEOUT_SEC 0
#define TIMEOUT_USEC 100000
#define LOG_FILE "/var/log/stdof_log.txt"


/* Macro */
#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

/* Prototype */
int create_named_pipe(int *);
int monitor_input_devices(struct udev_monitor *, 
                          const char *, int *);
void segway_status_update(struct segway_struct *, int, struct wwvi_js_event *, long int);
void manage_remote_joystick();
void redirect_stdof_message(int, int, char *);
int net_init(int *, struct sockaddr_in *, int);
void can_message_handle(struct can_frame *, int);
void message_log(const char *, const char *);
int copy_log_file(void);

int main()
{
  /* SocketCan interface */
  struct sockaddr_can addr;
  struct can_frame frame;
  struct ifreq ifr;
  int socket_can = -1;
  
  /* SocketNet interface */
/*  int socket_server = -1;
  int socket_client = -1;
  struct sockaddr_in server_address;
  struct sockaddr_in client_address;
  socklen_t client_length = sizeof(client_address);
  unsigned char net_buffer[255];*/

  /* Init Can Interface */
  /* The can is mandatory for this program */
  if(can_init(&socket_can, &addr, &ifr, 1, 0) == -1)
  {
    message_log("can_init", strerror(errno));
    perror("can_init");

    return 1;
  }
  else
  {
    message_log("can_init", "Init Can\t[OK]");
    printf("Init Can\t[OK]\n");
  }
  
  /* Joystick interface */
  struct wwvi_js_event jse;
  int joy_local = -1;

  /* Udev   */
  struct udev *udev;
  struct udev_monitor *udev_mon;
  int udev_fd = -1;
  int pid = -1;

  /* Segway */
  struct segway_struct segway_status;

  /* Pc interface */
  int rs232_device = -1;
  const unsigned char header = 0x024;
  unsigned char rs232_id;
  char rs232_buffer[255];
  
  /* Generic Variable */
  int done = 0;  // for the while in main loop
  int bytes_read;  // to check how many bytes has been read
  int bytes_sent;  // to check how many byte has been write

  int select_result = -1;  // value returned frome select()
  int nfds = 0;  // fd to pass to select()
  fd_set rd, wr, er; // structure for select()

  message_log("stdof", "Initializing stdof. . .");
  printf("Initializing stdof. . .\n");

  /* Peripheral initialization */

  /* Init Joystick */
  memset(&jse, 0, sizeof(struct wwvi_js_event));
  joy_local = open_joystick(JOY_LOCAL_NAME);

  if(joy_local < 0)
  {
    message_log("open_joystick(local)", strerror(errno));
    perror("open_joystick(local)");
  }
  else
  {
    pid = fork();

    if(pid == -1)
    {
      message_log("Remote Joystick", strerror(errno));
      perror("Remote Joystick");
    }
    else if(pid == 0)
      manage_remote_joystick();
    
    message_log("open_joystick", "Find Local Joystick\t[OK]");
    printf("Find Local Joystick\t[OK]\n");
  }

  /* Init Monitor for joystick or usb storage device  */
  udev = udev_new();

  if(!udev)
  {
    message_log("udev_new", "Init udev\t[FAIL]");
    printf("Can't create udev\n");
  }
  else 
  {
    udev_mon = udev_monitor_new_from_netlink(udev, "udev");
    udev_monitor_filter_add_match_subsystem_devtype(udev_mon, "input", NULL); // for joystick
    udev_monitor_filter_add_match_subsystem_devtype(udev_mon, "block", "partition"); // for storage device
    udev_monitor_enable_receiving(udev_mon);
    udev_fd = udev_monitor_get_fd(udev_mon);

    message_log("udev_new", "Init udev\t[OK]");
    printf("Init udev\t[OK]\n");
  }

  /* Init RS232 Interface */
  rs232_device = com_open("/dev/ttyO2", 115200, 'N', 8, 1);
   
  if(rs232_device == -1)
  {
    message_log("com_open", strerror(errno));
    perror("com_open");
  }
  else
  {
    message_log("com_open", "Init RS232\t[OK]");
    printf("Init RS232\t[OK]\n");
  }
  
  message_log("stdof", "Run main program. . .");
  printf("Run main program. . .\n");

  /* Init Network */
  /*if(net_init(&socket_server, &server_address, 9010) == -1)
  {
    message_log("net_uinit", strerror(errno));
    perror("net_init");
  }
  else
  {
    message_log("net_init", "Init Net\t[OK]");
    printf("Init Net\t[OK]\n");
  }*/

  while(!done)
  {
    fflush(stdout);

    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);

    if(socket_can > 0)
    {
      FD_SET(socket_can, &rd);
      nfds = max(nfds, socket_can);
    }
	
    if(joy_local > 0)
    {
      FD_SET(joy_local, &rd);
      nfds = max(nfds, joy_local);
    }

    if(udev_fd > 0)
    {
      FD_SET(udev_fd, &rd);
      nfds = max(nfds, udev_fd);
    }

    if(rs232_device > 0)
    {
      FD_SET(rs232_device, &rd);
      nfds = max(nfds, rs232_device);
    }
	
/*    if(socket_server > 0)
    {
      FD_SET(socket_server, &rd);
      nfds = max(nfds, socket_server);
    }*/
	
    select_result = select(nfds + 1, &rd, NULL, NULL, NULL);

    if(select_result == -1 && errno == EAGAIN)
    {
      perror("select");
      continue;
    }

    if(select_result == -1)
    {
      message_log("stdof", strerror(errno));
      perror("main:");

      return 1;
    }

    /* Manage can message */
    if(socket_can > 0)
    {
      if(FD_ISSET(socket_can, &rd))
      {
        // Read a message back from the CAN bus
        bytes_read = read(socket_can, &frame, sizeof(frame));

        if(bytes_read < 0)
        { 
          message_log("can raw socket read", strerror(errno));
          perror("can raw socket read");
        }
        else if(bytes_read < sizeof(struct can_frame))
        {
          message_log("can raw socket read", "Incomplete CAN frame");
          printf("read: incomplete CAN frame\n");
        }
      
        can_message_handle(&frame, rs232_device);
       
        // get interface name of the received CAN frame
        //  ifr.ifr_ifindex = addr.can_ifindex;
        //ioctl(socket_can, SIOCGIFNAME, &ifr);
        //printf("Received a CAN frame from interface %s\n", ifr.ifr_name);
        continue;
      } // end if(FD_ISSET(socket_can, &rd))
    }
	
    /* Monitor input devices */
    if(udev_fd > 0)
    {
      if(FD_ISSET(udev_fd, &rd))
      {
        monitor_input_devices(udev_mon, JOY_LOCAL_NAME, &joy_local);
        continue;
      }
    }

    /* Manage joystick command */
    if((joy_local > 0))
    {
      if(FD_ISSET(joy_local, &rd))
      {
        bytes_read = get_joystick_status(&joy_local, &jse);

        if(bytes_read <= 0)
        {
          message_log("joystick", strerror(errno));
          perror("get_joystick_status");
    
          continue;
        }  // end if(bytes_read <= 0)
 
        if( rs232_device > 0)
        {
          rs232_buffer[0] = 0x24;
          rs232_buffer[1] = (0x08 << 3) | 0x05;
          rs232_buffer[2] = 0x80;
          rs232_buffer[3] = jse.stick_x >> 8;
          rs232_buffer[4] = jse.stick_x;
          rs232_buffer[5] = jse.stick_y >> 8;
          rs232_buffer[6] = jse.stick_y;
          rs232_buffer[7] = jse.stick_z >> 8;
          rs232_buffer[8] = jse.stick_z;
          rs232_buffer[9] = jse.button[0];
          rs232_buffer[10] = jse.button[1];

          //sprintf(rs232_buffer, "%x%x%x%x%d%d%d%d%d", 0x24, 0x06, 0x00, 0x05, jse.stick_x, jse.stick_y, jse.stick_z, jse.button[0], jse.button[1]);
          //printf("%s\n", rs232_buffer);
          //can_frame_to_write.can_id = 0x632;
          //can_frame_to_write.can_dlc = bytes_read;
          //memcpy(can_frame_to_write.data, rs232_buffer, can_frame_to_write.can_dlc);
          //write(socket_can, &can_frame_to_write, sizeof(can_frame_to_write));
          write(rs232_device, rs232_buffer, 12); 
        }
        //printf("X: %d\tY: %d\tZ: %d\nbutton1: %d\tbutton2: %d\n", jse.stick_x, jse.stick_y, jse.stick_z, jse.button[0], jse.button[1]);
        //printf("Message from parent: %d\n", jse.stick_z);

        continue;
      }//end if(joy_local > 0)
    }
	
    /* Redirect stdof message */
    if(rs232_device > 0)
    {
      if(FD_ISSET(rs232_device, &rd))
      {
        bytes_read = read(rs232_device, rs232_buffer, 255);

        if(bytes_read > 0)
        {
          redirect_stdof_message(socket_can, bytes_read, rs232_buffer);
		  
          continue;
        }
      }
    }
  }  // end while(!= done)

  return 0;
}

int monitor_input_devices(struct udev_monitor *udev_mon, 
                          const char *joystick_path, int *joy_local)
{
  struct udev_device *udev_dev;
  const char *udev_node;
  const char *udev_subsystem;
  int pid = -1;

  udev_dev = udev_monitor_receive_device(udev_mon);

  if(udev_dev)
  {
    udev_node =  udev_device_get_devnode(udev_dev);
    udev_subsystem = udev_device_get_subsystem(udev_dev);

    if(udev_node && !strcmp(udev_subsystem, "input")) 
    {
      if(!strcmp(udev_node, joystick_path))
      {
        if((!strcmp(udev_device_get_action(udev_dev),"add")) && (*joy_local < 0))
	{
	  // add joystick
          if(*joy_local > 0)
          {
            message_log("udev", "Cannot connect second joystick");
            printf("Cannot connect second joystick");
            return 0;
          }  
   
          *joy_local = open_joystick(joystick_path);

          if(*joy_local < 0)
            perror("open_joystick");

          message_log("stdof", "Joystick added");
	  printf("stdof: Joystick added\n");

          // send joystick command over network
          pid = fork();

          if(pid == -1)
          {
            message_log("Init Remote Joystick", strerror(errno));
            perror("Init Remote Joystick");
          }
         else if(pid == 0)
            manage_remote_joystick();
         
	}
        else if((!strcmp(udev_device_get_action(udev_dev),"remove")) && (*joy_local > 0))
        {
          // remove joystick
          *joy_local = -1;

          message_log("stdof", "Joystick removed");
          printf("stdof: Joystick removed\n");
        }
      }
    }  // end if(udev_node && !strcmp(udev_subsystem, "input")) 
    else if(udev_node && !strcmp(udev_subsystem, "block")) 
    {
      if(!strcmp(udev_device_get_action(udev_dev),"add"))
      {
        message_log("stdof", "Find storage device");
        printf("udev: Find storage device %s\n", udev_node);

        // mount usb device
        if(mount(udev_node, "/media/usb", "vfat", MS_NOATIME, NULL))
        {
          message_log("mount", strerror(errno));
          perror("mount");
          return -1;
        }
        else
        {
          message_log("mount", "Drive mounted");
          printf("mount: Drive mounted\n");
        }

        if(copy_log_file() < 0)
        {
          message_log("copy_log_file", strerror(errno));
          perror("copy_log_file");
        }
        else
        {
          message_log("copy_log_file", "Log copied to usb device");
          printf("copy_log_file: Log copied to usb device\n");
        }

        // unmount usb device
        if(umount("/media/usb") < 0)
        {
          message_log("umount", strerror(errno));
          perror("umount");
        }
        else
        {
          message_log("umount", "Drive unmounted");
          printf("mount: Drive unmounted\n");
        }
      }
      else if(!strcmp(udev_device_get_action(udev_dev),"remove"))
      {
        message_log("stdof", "Storage device removed");
        printf("udev: Storage device %s removed\n", udev_node);
      }
    }
    udev_device_unref(udev_dev);
  }

  return 0;
}

void segway_status_update(struct segway_struct *segway_status, int socket_can,  struct wwvi_js_event *jse, long int joy_max_value)
{
  static int segway_previouse_state = 0;
  int bytes_sent = -1;

  switch(segway_status->operational_state)
  {
    case CCU_INIT:
      if(segway_previouse_state != segway_status->operational_state)
      {
        message_log("stdof", "Segway CCU Init");
        printf("Segway CCU Init\n");

        segway_previouse_state = segway_status->operational_state;
      }
      break;

    case PROPULSION_INIT:
      if(segway_previouse_state != segway_status->operational_state)
      {
        message_log("stdof", "Segway Propulsion Init");
        printf("Segway Propulsion Init\n");

        segway_previouse_state = segway_status->operational_state;
      }
      break;

    case CHECK_STARTUP:
      if(segway_previouse_state != segway_status->operational_state)
      {
        message_log("stdof", "Segway Check Startup Issue");
        printf("Segway Check Startup Issue\n");

        segway_previouse_state = segway_status->operational_state;
      }
      break;

    case SEGWAY_STANDBY:  //standby mode
      if(segway_previouse_state != segway_status->operational_state)
      {
        message_log("stdof", "Segway in Standby Mode");
        printf("Segway in Standby Mode\n");

        segway_previouse_state = segway_status->operational_state;
      }

      // I can only send a tractore request
      if(jse->button[0])
      {
        bytes_sent = segway_configure_operational_mode(socket_can, SEGWAY_TRACTOR_REQ);
      
        if(bytes_sent == -1)
        { 
          message_log("segway_configure_operational_mode to tractor", strerror(errno));
          perror("segway_configure_operational_mode");
        }

        // get new state
        bytes_sent = segway_configure_none(socket_can, 0x00);

        break;
      }
      break;

    case SEGWAY_TRACTOR:
      if(segway_previouse_state != segway_status->operational_state)
      {
        message_log("stdof", "Segway in Tractor Mode");
        printf("Segway in Tractor Mode\n");

        segway_previouse_state = segway_status->operational_state;
      }

      if(jse->button[1])
      {
        bytes_sent = segway_configure_operational_mode(socket_can, SEGWAY_STANDBY_REQ);

        if(bytes_sent == -1)
        {
          message_log("segway_configure_operational_mode to standby", strerror(errno));
          perror("segway_configure_operational_mode");
        }

        // get new state
        bytes_sent = segway_configure_none(socket_can, 0x00);
	      
        break;
      }
            
      if(jse->stick_z > 0)
        bytes_sent = segway_motion_set(socket_can, -jse->stick_y, jse->stick_x, joy_max_value);
      else
        bytes_sent = segway_motion_set(socket_can, 0, 0, joy_max_value);
	      
      if(bytes_sent < 0)
      {
        message_log("segway_motion_set", strerror(errno));
        perror("segway_motion_set");
      }

      break;

    case DISABLE_POWER:
      if(segway_previouse_state != segway_status->operational_state)
      {
        message_log("stdof", "Segway Disable Power");
        printf("Segway Disable Power\n");

        segway_previouse_state = segway_status->operational_state;
      }
      break;

    default:
      if(segway_previouse_state != segway_status->operational_state)
      {
        message_log("stdof", "Segway Uknown State");
        printf("Segway Unknown State\n");

        segway_previouse_state = segway_status->operational_state;
      }
      break;
  } //end switch 
}

void manage_remote_joystick()
{
  int exec_return = -1;

  printf("Send message to joystick ....\n");
  fflush(stdout);

  exec_return = execlp("./joystick.sh", "joystick.sh", (char *) 0, (char *) 0);

  if(exec_return == -1)
  {
    perror("exec:");
  }

  printf("Stop send command to joystick!\n");
  fflush(stdout);

  exit(0);
}

void redirect_stdof_message(int socket_can, int bytes_read, char *message)
{
  static struct can_frame can_frame_to_write;
  static unsigned char rs232_state = 0;
  static unsigned char rs232_data_count = 0;
  static int count = 0;
  
  unsigned char rs232_data_read = 0;
  int length_to_write = 0;
  int i;

  while(rs232_data_read < bytes_read)
  { 
    switch(rs232_state)
    {
      case 0: // waiting for the header
        if(message[rs232_data_read] == 0x24)
          rs232_state++;

        rs232_data_read++;
 
        if(rs232_data_read >= bytes_read)
          break;

      case 1: //read message length and first address byte
        rs232_data_count = message[rs232_data_read] >> 3;

        if(rs232_data_count == 0)
        {
          rs232_state = 0;
          break;
        }

        can_frame_to_write.can_id = ((__u32)message[rs232_data_read] & 0x0007) << 8;
        rs232_state++;
        rs232_data_read++;
                
        if(rs232_data_read >= bytes_read)
          break;

      case 2: //read second address byte
        can_frame_to_write.can_id |= message[rs232_data_read];
        rs232_state++;
        rs232_data_read++;
               
        if(rs232_data_read >= bytes_read) 
          break;

      case 3: //read data and send over can bus
        // if the number of data bytes is less then the whole message length decleared
        // then resize the expected data for the next loop

        if((bytes_read - rs232_data_read) <= rs232_data_count)
        {
          // The max data length allowed for one frame is 8
          if((bytes_read - rs232_data_read) > 8)
            length_to_write = 8;
          else
            length_to_write = bytes_read - rs232_data_read;
        }
        else
        {
          // if there's more data then we expected, get one message and return
          // to the start to find the header for the next one.
          if(rs232_data_count > 8)
            length_to_write = 8;
          else
            length_to_write = rs232_data_count;
        }
 
        can_frame_to_write.can_dlc = length_to_write;

        memcpy(can_frame_to_write.data, &message[rs232_data_read], length_to_write);
        rs232_data_count -= length_to_write;
        rs232_data_read += length_to_write;

        // if I read all data, return to the start to process the rest of the message
        if(rs232_data_count == 0)
          rs232_state = 0;

        if((rs232_data_count == 0) || (length_to_write == 8) || (rs232_data_read == bytes_read))
	{
          count += length_to_write;
          printf("[%s] [%x] [%x]", "Write", can_frame_to_write.can_id, can_frame_to_write.can_dlc);

          for(i = 0; i < can_frame_to_write.can_dlc; i++)
            printf(" [%x]", can_frame_to_write.data[i]);

          printf("%i", count);
          printf("\n");
          fflush(stdout);

          write(socket_can, &can_frame_to_write, sizeof(can_frame_to_write));
        }
        break;

      default:
        rs232_state = 0;
        rs232_data_count = 0;
        break;
    }  // end switch
  } // end while

  //can_frame_to_write.can_id = 0x632;
  //can_frame_to_write.can_dlc = bytes_read;
  //memcpy(can_frame_to_write.data, message, can_frame_to_write.can_dlc);
  //write(socket_can, &can_frame_to_write, sizeof(can_frame_to_write));
}

int net_init(int *socket_net, struct sockaddr_in *server_addr, int portnumber)
{
  *socket_net = socket(AF_INET, SOCK_STREAM, 0);

  if(*socket_net < 0)
    return -1;

  bzero((char *) server_addr, sizeof(*server_addr));
  server_addr->sin_family = AF_INET;
  server_addr->sin_port = htons(portnumber);

  if(bind(*socket_net, (struct sockaddr *)server_addr, sizeof(*server_addr)) < 0)
    return -1;

  listen(*socket_net, 1);

  return 0;
}

void can_message_handle(struct can_frame *frame, int rs232_device)
{
  int i = 0;
  static int count = 0;
  char rs232_header[3];

  if(frame->can_id & CAN_RTR_FLAG)
    printf("Request flag found!\n");
  
  switch(frame->can_id)
  {
    case 0x502: 
    case 0x503:
    case 0x504:
    case 0x505: 
    case 0x506:
    case 0x507:
    case 0x508:
    case 0x509:
    case 0x50a:
    case 0x50b:
    case 0x50c:
    case 0x50d:
    case 0x50e:
    case 0x50f:
    case 0x510:
    case 0x511:
    case 0x512:
    case 0x513:
    case 0x514:
    case 0x515:
    case 0x516:
    case 0x517:
    case 0x518:
    case 0x519:
    case 0x51a:
    case 0x51b:
    case 0x51c:
    case 0x51d:
    case 0x51e:
    case 0x51f:
    case 0x520:
    case 0x521:
    case 0x522:
    case 0x523:
    case 0x524:
    case 0x525:
    case 0x526:
    case 0x527:
    case 0x528:
    case 0x529:
    case 0x52a:
    case 0x52b:
      /* Update the segway flags */
      segway_config_update(frame);
	    
      /* do
         {
           fault_return = segway_config_decode_arch_fault(segway_status.fault_status_word1, fault_message);
	      
           if(fault_return >= 0)
	   {
             message_log("segway_config_fault:", fault_message);
             printf("Fault: %s\n", fault_message);
	    }
          } while(fault_return > 0);
	    
          do
          {
            fault_return = segway_config_decode_critical_fault(segway_status.fault_status_word1, fault_message);

            if(fault_return >= 0)
            {
              message_log("segway_config_fault:", fault_message);
              printf("Fault: %s\n", fault_message);
	    }
          } while(fault_return > 0);

          do
          {
            fault_return = segway_config_decode_comm_fault(segway_status.fault_status_word2, fault_message);

            if(fault_return >= 0)
	    {
              message_log("segway_config_fault:", fault_message);
              printf("Fault: %s\n", fault_message);
	    }
          } while(fault_return > 0);

          do
          {
            fault_return = segway_config_decode_internal_fault(segway_status.fault_status_word2, fault_message);

            if(fault_return >= 0)
            {
              message_log("segway_config_fault:", fault_message);
              printf("Fault: %s\n", fault_message);
            }
          } while(fault_return > 0);

          do
          {
            fault_return = segway_config_decode_sensors_fault(segway_status.fault_status_word3, fault_message);

            if(fault_return >= 0)
            {
              message_log("segway_config_fault:", fault_message);
              printf("Fault: %s\n", fault_message);
            }
          } while(fault_return > 0);

          do
          {
            fault_return = segway_config_decode_bsa_fault(segway_status.fault_status_word3, fault_message);

            if(fault_return >= 0)
            {
              message_log("segway_config_fault:", fault_message);
              printf("Fault: %s\n", fault_message);
            }
          } while(fault_return > 0);

          do
          {
            fault_return = segway_config_decode_mcu_fault(segway_status.fault_status_word4, fault_message);

            if(fault_return >= 0)
            {
              message_log("segway_config_fault:", fault_message);
              printf("Fault: %s\n", fault_message);
            }
          } while(fault_return > 0);

          do
          {
            fault_return = segway_config_decode_mcu_message(segway_status.mcu_0_fault_status, fault_message);

            if(fault_return >= 0)
            {
              message_log("segway mcu0", fault_message);
              printf("segway mcu0: %s\n", fault_message);
            }
          } while(fault_return > 0);

          do
          {
            fault_return = segway_config_decode_mcu_message(segway_status.mcu_1_fault_status, fault_message);

            if(fault_return >= 0)
            {
              message_log("segway mcu1", fault_message);
              printf("segway mcu1: %s\n", fault_message);
            }
          } while(fault_return > 0);

          do
          {
            fault_return = segway_config_decode_mcu_message(segway_status.mcu_2_fault_status, fault_message);

            if(fault_return >= 0)
            {
              message_log("segway mcu2", fault_message);
              printf("segway mcu2: %s\n", fault_message);
            }
          } while(fault_return > 0);

          do
          {
            fault_return = segway_config_decode_mcu_message(segway_status.mcu_3_fault_status, fault_message);

            if(fault_return >= 0)
            {
              message_log("segway mcu3", fault_message);
              printf("segway mcu3: %s\n", fault_message);
            }
          } while(fault_return > 0);
          printf("Operational state: %08lx\n", segway_status.operational_state);
          printf("Linear velocity: %08lx\n", segway_status.linear_vel_mps);
          printf("Linear position: %08lx\n", segway_status.linear_pos_m);
          printf("Front Batt1 SOC: %08lx\n", segway_status.front_base_batt_1_soc);
          printf("Front Batt2 SOC: %08lx\n", segway_status.front_base_batt_2_soc);
          printf("Rear Batt1 SOC: %08lx\n", segway_status.rear_base_batt_1_soc);
          printf("Rear Batt2 SOC: %08lx\n", segway_status.rear_base_batt_2_soc);
          printf("Front Batt1 Temp: %08lx\n", segway_status.front_base_batt_1_temp_degC);
          printf("Front Batt2 Temp: %08lx\n", segway_status.front_base_batt_2_temp_degC);
          printf("Rear Batt1 Temp: %08lx\n", segway_status.rear_base_batt_1_temp_degC);
          printf("Rear Batt2 Temp: %08lx\n", segway_status.rear_base_batt_2_temp_degC);*/
          break;

    default:
      printf("[%s] [%x] [%x] ", "Read", frame->can_id, frame->can_dlc);
	  
      for(i = 0; i < frame->can_dlc; i++)
      {
        printf("[%x] ", frame->data[i]);
        count++;
      }

      printf(" %d\n", count);

      if(rs232_device > 0)
      {
        rs232_header[0] = 0x24;
        rs232_header[1] = (frame->can_dlc << 3) | (frame->can_id >> 8);
        rs232_header[2] = frame->can_id;
        write(rs232_device, rs232_header, sizeof(rs232_header));
        write(rs232_device, frame->data, frame->can_dlc);
      }          
      break;
  }  // end switch
}

void message_log(const char *scope, const char *message)
{
  char buffer[32];
  struct tm *ts;
  size_t last;
  time_t timestamp = time(NULL);
  FILE *file = NULL;

  // Init Log File
  file = fopen(LOG_FILE, "a");
 
  if(!file)
  {
    perror("logfile fopen:");
    return;
  }

  ts = localtime(&timestamp);
  last = strftime(buffer, 32, "%b %d %T", ts);
  buffer[last] = '\0';

  fprintf(file, "[%s]%s: %s\n", buffer, scope, message);

  fclose(file);
}

int copy_log_file()
{
  FILE *log = NULL;
  FILE *disk = NULL;
  char ch;

  disk = fopen("/media/usb/stdof_log.txt", "w");

  if(!disk)
    return -1;

  log = fopen(LOG_FILE, "r");

  if(!log)
  {
    fclose(disk);
    printf("log file:\n");
    return -1;
  }

  while((ch = fgetc(log)) != EOF)
  {
    if(feof(log))
      break;

    fputc(ch, disk);
  }
  
  fclose(log);
  fclose(disk);
  
  return 0;
}
