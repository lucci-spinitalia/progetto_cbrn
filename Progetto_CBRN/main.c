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

#include <locale.h>

#include <string.h>

#include <linux/joystick.h>

#include <sys/mount.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <netinet/in.h>

#include "socket_can_interface.h"
#include "pc_interface_udp.h"

#define CCU_ADDRESS "192.168.1.102"
#define CCU_PORT 8010
#define PC_INTERFACE_PORT 8011

/* Macro */
#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

/* Prototype */
int redirect_stdof_message(int, int, struct pc_interface_udp_frame);
int can_message_handle(struct can_frame *, int); 

int main()
{
  /* SocketCan interface */
  struct sockaddr_can addr;
  struct can_frame frame;
  struct ifreq ifr;
  int socket_can = -1;
  
  /* Pc interface */
  int pc_interface_client = -1;
  struct pc_interface_udp_frame pc_interface_buffer_temp;
  struct sockaddr_in pc_interface_client_address;

  /* Generic Variable */
  int done = 0;  // for the while in main loop
  int bytes_read;  // to check how many bytes has been read
  int bytes_sent;

  int select_result = -1;  // value returned frome select()
  int nfds = 0;  // fd to pass to select()
  fd_set rd, wr, er; // structure for select()

  /* Peripheral initialization */

  /* Init Can Interface */
  /* The can is mandatory for this program */
  if(can_init(&socket_can, &addr, &ifr, 1, 0) == -1)
  {
    perror("can_init");
    return 1;
  }
  else
    printf("Init Can\t[OK]\n");

	
  /* Init Pc Interface */
  if(pc_interface_connect(&pc_interface_client, &pc_interface_client_address, PC_INTERFACE_PORT, CCU_ADDRESS, CCU_PORT) == 0)
    perror("error connection");
  else
    printf("Init pc interface client\t[OK]\n");
 
  printf("Run main program. . .\n");

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

      if(pc_interface_buffer_rx_empty == 0)
      {
        FD_SET(socket_can, &wr);
        nfds = max(nfds, socket_can);
      }
    }
	
    if(pc_interface_client > 0)
    {
      FD_SET(pc_interface_client, &rd);
      nfds = max(nfds, pc_interface_client);  

      if(pc_interface_buffer_tx_empty == 0)
      {
        FD_SET(pc_interface_client, &wr);
        nfds = max(nfds, pc_interface_client);
      }
    }

    select_result = select(nfds + 1, &rd, &wr, NULL, NULL);

    if(select_result == -1 && errno == EAGAIN)
    {
      perror("select");
      continue;
    }

    if(select_result == -1)
    {
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
          perror("can raw socket read");
        else if(bytes_read < sizeof(struct can_frame))
          printf("read: incomplete CAN frame\n");

        bytes_sent = can_message_handle(&frame, pc_interface_client);

        if(bytes_sent < 0)
          printf("Error on can_message_handle\n");
       
      } // end if(FD_ISSET(socket_can, &rd))

      if(FD_ISSET(socket_can, &wr))
      {
        bytes_read = pc_interface_unload_rx(&pc_interface_buffer_temp);

        if(bytes_read > 0)
          redirect_stdof_message(socket_can, bytes_read, pc_interface_buffer_temp);

      }
    }
    	
    /* Redirect stdof message */
    if(pc_interface_client > 0)
    {
      if(FD_ISSET(pc_interface_client, &rd))
      {
        bytes_read = pc_interface_read(pc_interface_client);

        if(bytes_read < 0)
          perror("Read from pc_interface_device");

      }

      if(FD_ISSET(pc_interface_client, &wr))
      {
        //printf("%i\n", rs232_buffer_tx_data_count);
        bytes_sent = pc_interface_send(pc_interface_client, &pc_interface_client_address);

        if(bytes_sent <= 0)
        {
          printf("Error on pc_interface_send\n");
          can_restart();
        }

      }  //if(FD_ISSET(pc_interface_client, &wr))
    }
  }  // end while(!= done)
  
  return 0;
}

int redirect_stdof_message(int socket_can, int bytes_read, struct pc_interface_udp_frame message)
{
  struct can_frame can_frame_to_write;
  unsigned char pc_interface_data_count = 0;
  
  unsigned char pc_interface_data_read = 0;
  int length_to_write = 0;
  int bytes_sent;

  if(message.param.header == 0x24)
  {
    can_frame_to_write.can_id = (((__u32)message.param.length & 0x0007) << 8) | message.param.id;
    
	pc_interface_data_count = message.param.length >> 3;
	
	// if the number of data bytes is less then the whole message length decleared
    // then resize the expected data for the next loop
    while(pc_interface_data_count > 0)
	{
      // The max data length allowed for one frame is 8
      if((pc_interface_data_count) > 8)
        length_to_write = 8;
      else
        length_to_write = pc_interface_data_count;

      can_frame_to_write.can_dlc = length_to_write;
	  
	  memcpy(can_frame_to_write.data, &message.param.data[pc_interface_data_read], length_to_write);
      pc_interface_data_count -= length_to_write;
      pc_interface_data_read += length_to_write;
  
      bytes_sent = write(socket_can, &can_frame_to_write, sizeof(can_frame_to_write));

      if(bytes_sent == -1)
            return -1;

      if(bytes_sent != sizeof(struct can_frame))
        printf("Error on write: bytes sent %i\n", bytes_sent);
	}
  }  //end while
  

  return 0;
}

int can_message_handle(struct can_frame *frame, int device)
{
  //static int count = 0;
  //int i;
  int bytes_sent;
  struct pc_interface_udp_frame udp_message;

  if(frame->can_id & CAN_RTR_FLAG)
    printf("Request flag found!\n");
  
  switch(frame->can_id)
  {  
    default:
      //printf("[%s] [%x] [%x] ", "Read", frame->can_id, frame->can_dlc);
	  
      /*for(i = 0; i < frame->can_dlc; i++)
      {
        count += 1;
        printf("[%x] ", frame->data[i]);
      }

      printf("%i\n", count);*/
      

      udp_message.param.header = 0x24;
      udp_message.param.length = (frame->can_dlc << 3) | (frame->can_id >> 8);
      udp_message.param.id = frame->can_id;
      memcpy(udp_message.param.data, frame->data, frame->can_dlc);

      bytes_sent = pc_interface_load_tx(udp_message);

      return bytes_sent;

      break;
  }  // end switch
}
