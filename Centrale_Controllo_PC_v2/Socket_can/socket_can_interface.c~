#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
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

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>

#include <linux/joystick.h>

#include <sys/mount.h>

#include <netinet/in.h>

// PF stands for Protocol Family
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

/* Prototype */
int can_init(int *, struct sockaddr_can *, struct ifreq *, int, int);
int net_init(int *, struct sockaddr_in *, int);
void manage_remote_joystick(int *pipefd);

/*************************************************************
 * loopback: 0 = disabled, 1 = enabled
 * recv_own_msgs: 0 = disabled, 1 = enabled
 *
 */
int can_init(int *socket_can, struct sockaddr_can *addr, struct ifreq *ifr, int loopback, int recv_own_msgs)
{
  // Create the socket
  *socket_can = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  if(*socket_can == -1)
    return -1;

  // Filter rules
  /*struct can_filter rfilter[2];

  rfilter[0].can_id = 0x123;
  rfilter[0].can_mask = CAN_SFF_MASK;
  rfilter[1].can_id = 0x200;
  rfilter[1].can_mask = 0x700;

  setsockopt(skt, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));*/

  // Set loopback option
  setsockopt(*socket_can, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

  // Set if receive own message or not
  setsockopt(*socket_can, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));

  // Locate the interface you wish to use
  strcpy(ifr->ifr_name, "can0");
  ioctl(*socket_can, SIOCGIFINDEX, ifr);

  // Select that CAN interface, and bind the socket to it.
  addr->can_family = AF_CAN;
  addr->can_ifindex = ifr->ifr_ifindex;

  if(bind(*socket_can, (struct sockaddr*)addr, sizeof(struct sockaddr)) == -1)
    return -1;

  return 0;
}


/*int net_init(int *socket_net, struct sockaddr_in *server_addr, int portnumber)
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
}*/



int main()
{

  /* SocketNet interface */
/*  int socket_server = -1;
  int socket_client = -1;
  struct sockaddr_in server_address;
  struct sockaddr_in client_address;
  socklen_t client_length = sizeof(client_address);
  unsigned char net_buffer[255];
*/

 
  /* Pc interface */
  int rs232_device = -1;
  const unsigned char header = 0x024;
  unsigned char rs232_id;
  unsigned char rs232_state = 0;
  unsigned char rs232_data_count = 0;
  unsigned char rs232_data_read = 0;
  struct can_frame can_frame_to_write;
  unsigned char rs232_buffer[255];
  

  // Init RS232 Interface
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

  /* Check if there's a  named pipe that handle the remote joystick */

  


/*  if(net_init(&socket_server, &server_address, 9010) == -1)
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
    if(rs232_device > 0)
    {
      FD_SET(rs232_device, &rd);
      nfds = max(nfds, rs232_device);
    }

    /* Read the usb joystick only if the remote connection is down */

    /* Manage RS232 */
    if(rs232_device > 0)
    {
      if(FD_ISSET(rs232_device, &rd))
      {
        bytes_read = read(rs232_device, rs232_buffer, 255);

        if(bytes_read > 0)
        {
          rs232_buffer[bytes_read] = 0;

          rs232_data_read = 0;

          while(rs232_data_read < bytes_read)
          { 
            switch(rs232_state)
            {
              case 0: // waiting for the header
	        printf("%d", rs232_state);
		fflush(stdout);

                if(rs232_buffer[rs232_data_read] == header)
                  rs232_state++;

                rs232_data_read++;
 
                if(rs232_data_read >= bytes_read)
	          break;

              case 1: //read first address byte
                printf("%d", rs232_state);
		fflush(stdout);

                can_frame_to_write.can_id = ((__u32)rs232_buffer[rs232_data_read] & 0x0007) << 8;
                rs232_state++;
                rs232_data_read++;
                
                if(rs232_data_read >= bytes_read)
                  break;

              case 2: //read second address byte
                printf("%d", rs232_state);
		fflush(stdout);

                can_frame_to_write.can_id |= rs232_buffer[rs232_data_read];
                rs232_state++;
                rs232_data_read++;
               
                if(rs232_data_read >= bytes_read) 
                  break;
 
              case 3: //read lenght byte
		printf("%d", rs232_state);
		fflush(stdout);

                can_frame_to_write.can_dlc = rs232_buffer[rs232_data_read];

                if(can_frame_to_write.can_dlc == 0)
                  rs232_state = 0;
                else
                  rs232_state++;

                rs232_data_read++;

                if(rs232_data_read >= bytes_read)
                  break;

              case 4: //read data and send over can bus
		printf("%d", rs232_state);
		fflush(stdout);

                if((bytes_read - rs232_data_read) <= can_frame_to_write.can_dlc)
                {
                  memcpy(can_frame_to_write.data, &rs232_buffer[rs232_data_read], bytes_read - rs232_data_read);
                  rs232_data_count = bytes_read - rs232_data_read;
                  rs232_data_read = bytes_read;
                }
                else
                {
                  memcpy(can_frame_to_write.data, &rs232_buffer[rs232_data_read], can_frame_to_write.can_dlc);
                  rs232_data_count += can_frame_to_write.can_dlc;
                  rs232_data_read += can_frame_to_write.can_dlc;
                }

                if(rs232_data_count == can_frame_to_write.can_dlc)
	        {
                  printf(":Write:%s:%d\n",can_frame_to_write.data, rs232_data_count);
                  fflush(stdout);

                  rs232_state = 0;
                  rs232_data_count = 0;

                  write(socket_can, &can_frame_to_write, sizeof(can_frame_to_write));
                }
                break;

              default:
                rs232_state = 0;
                rs232_data_count = 0;
                break;
            }
          }
          //can_frame_to_write.can_id = 0x632;
          //can_frame_to_write.can_dlc = bytes_read;
          //memcpy(can_frame_to_write.data, rs232_buffer, can_frame_to_write.can_dlc);
          //write(socket_can, &can_frame_to_write, sizeof(can_frame_to_write));
        }
        continue;
      }
    }

  }

  return 0;
}
