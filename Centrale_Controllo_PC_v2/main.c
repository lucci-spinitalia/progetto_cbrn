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
#include "./Rs232/rs232.h"

#define TIMEOUT_SEC 0
#define TIMEOUT_USEC 100000
#define RS232_BUFFER_SIZE 65536
#define RS232_MAX_TX_LENGTH 1024

/* Macro */
#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

/* Prototype */
int redirect_stdof_message(int, int, unsigned char *);
int can_message_handle(struct can_frame *, int);
int rs232_load_tx(unsigned char *data, unsigned int data_length);
int rs232_unload_rx(unsigned char *data);
int rs232_write(int rs232_device);
int rs232_read(int rs232_device);
int rs232_buffer_tx_get_space(void);
int rs232_buffer_rx_get_space(void);

char rs232_buffer_tx[RS232_BUFFER_SIZE];
unsigned int rs232_buffer_tx_ptr_wr;
unsigned int rs232_buffer_tx_ptr_rd;
unsigned char rs232_buffer_tx_empty;
unsigned char rs232_buffer_tx_full;
unsigned char rs232_buffer_tx_overrun;
unsigned int rs232_buffer_tx_data_count;  

char rs232_buffer_rx[RS232_BUFFER_SIZE];
unsigned int rs232_buffer_rx_ptr_wr;
unsigned int rs232_buffer_rx_ptr_rd;
unsigned char rs232_buffer_rx_empty;
unsigned char rs232_buffer_rx_full;
unsigned char rs232_buffer_rx_overrun;
unsigned int rs232_buffer_rx_data_count;  

int main()
{
  /* SocketCan interface */
  struct sockaddr_can addr;
  struct can_frame frame;
  struct ifreq ifr;
  int socket_can = -1;
  
  /* Pc interface */
  int rs232_device = -1;
  unsigned char rs232_buffer_temp[RS232_BUFFER_SIZE];
  int bytes_sent;

  /* Generic Variable */
  int done = 0;  // for the while in main loop
  int bytes_read;  // to check how many bytes has been read

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

  /* Init RS232 Interface */
  rs232_device = com_open("/dev/ttyO2", 115200, 'N', 8, 1);
   
  if(rs232_device == -1)
    perror("com_open");
  else
    printf("Init RS232\t[OK]\n");
  
  // Init circular buffer
  rs232_buffer_tx_ptr_wr = 0;
  rs232_buffer_tx_ptr_rd = 0;
  rs232_buffer_tx_empty = 1;
  rs232_buffer_tx_full = 0;
  rs232_buffer_tx_overrun = 0;
  rs232_buffer_tx_data_count = 0;

  rs232_buffer_rx_ptr_wr = 0;
  rs232_buffer_rx_ptr_rd = 0;
  rs232_buffer_rx_empty = 1;
  rs232_buffer_rx_full = 0;
  rs232_buffer_rx_overrun = 0;
  rs232_buffer_rx_data_count = 0;

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

      if(rs232_buffer_rx_empty == 0)
      {
        FD_SET(socket_can, &wr);
        nfds = max(nfds, socket_can);
      }
    }
	
    if(rs232_device > 0)
    {
      FD_SET(rs232_device, &rd);
      nfds = max(nfds, rs232_device);  

      if(rs232_buffer_tx_empty == 0)
      {
        FD_SET(rs232_device, &wr);
        nfds = max(nfds, rs232_device);
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

        bytes_sent = can_message_handle(&frame, rs232_device);

        if(bytes_sent < 0)
          printf("Error on can_message_handle\n");
       
      } // end if(FD_ISSET(socket_can, &rd))

      if(FD_ISSET(socket_can, &wr))
      {
        bytes_read = rs232_unload_rx(rs232_buffer_temp);

        if(bytes_read > 0)
          redirect_stdof_message(socket_can, bytes_read, rs232_buffer_temp);
      }
    }
    	
    /* Redirect stdof message */
    if(rs232_device > 0)
    {
      if(FD_ISSET(rs232_device, &rd))
      {
        bytes_read = rs232_read(rs232_device);

        if(bytes_read < 0)
          perror("Read from rs232_device");

      } //if(FD_ISSET(rs232_device, &rd))

      if(FD_ISSET(rs232_device, &wr))
      {
        //printf("%i\n", rs232_buffer_tx_data_count);
        bytes_sent = rs232_write(rs232_device);

        if(bytes_sent <= 0)
          printf("Error on rs232_write");

      }  //if(FD_ISSET(rs232_device, &wr))
    }
  }  // end while(!= done)

  return 0;
}

int rs232_buffer_tx_get_space(void)
{
  return (RS232_BUFFER_SIZE - rs232_buffer_tx_data_count);
}

int rs232_buffer_rx_get_space(void)
{
  return (RS232_BUFFER_SIZE - rs232_buffer_rx_data_count);
}

int rs232_load_tx(unsigned char *data, unsigned int data_length)
{
  int i;

  if(rs232_buffer_tx_full)
  {
    rs232_buffer_tx_overrun = 1;
    return -1;
  }

  if(rs232_buffer_tx_get_space() < data_length)
  {
    rs232_buffer_tx_full = 1;
    rs232_buffer_tx_overrun = 1;
    return -1;
  }

  for(i = 0; i < data_length; i++)
  {
    rs232_buffer_tx[rs232_buffer_tx_ptr_wr] = data[i];

    rs232_buffer_tx_data_count ++;
    rs232_buffer_tx_ptr_wr ++;

    if(rs232_buffer_tx_ptr_wr == RS232_BUFFER_SIZE)
      rs232_buffer_tx_ptr_wr = 0;
  }

  rs232_buffer_tx_empty = 0;

  if(rs232_buffer_tx_data_count == RS232_BUFFER_SIZE)
    rs232_buffer_tx_full = 1;

  return 1;
}

int rs232_unload_rx(unsigned char *data)
{
  int length_to_write = 0;

  if(rs232_buffer_rx_empty)
    return 0;

  rs232_buffer_rx_full = 0;
 
  if(rs232_buffer_rx_ptr_rd < rs232_buffer_rx_ptr_wr)
    length_to_write = (rs232_buffer_rx_ptr_wr - rs232_buffer_rx_ptr_rd);
  else
    length_to_write = (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_rd);

  memcpy(data, &rs232_buffer_rx[rs232_buffer_rx_ptr_rd], length_to_write);

  rs232_buffer_rx_data_count -= length_to_write;

  if(rs232_buffer_rx_data_count == 0)
    rs232_buffer_rx_empty = 1;

  rs232_buffer_rx_ptr_rd += length_to_write;

  if(rs232_buffer_rx_ptr_rd == RS232_BUFFER_SIZE)
    rs232_buffer_rx_ptr_rd = 0;

//  printf("\nempty: %i, data_count: %i\n", rs232_buffer_rx_empty,rs232_buffer_rx_data_count);
//  printf("full: %i, rd pointer: %i\n", rs232_buffer_rx_full, rs232_buffer_rx_ptr_rd);
//  printf("\n");

  return length_to_write;
}

int rs232_read(int rs232_device)
{
  int bytes_read = -1;

  if(rs232_buffer_rx_full)
  {
    rs232_buffer_rx_overrun = 1;
    return -1;
  }

  if(rs232_device > 0)
  {
    // I want to read as many bytes as possible, but I have to manage the
    // circular buffer. So I can read only the data before restart the
    // buffer.
    bytes_read = read(rs232_device, &rs232_buffer_rx[rs232_buffer_rx_ptr_wr], (RS232_BUFFER_SIZE - rs232_buffer_rx_ptr_wr));

    if(bytes_read > 0)
    {
      rs232_buffer_rx_empty = 0;
      rs232_buffer_rx_data_count += bytes_read;
 
      if(rs232_buffer_rx_data_count == RS232_BUFFER_SIZE)
        rs232_buffer_rx_full = 1;

      rs232_buffer_rx_ptr_wr += bytes_read;

      if(rs232_buffer_rx_ptr_wr == RS232_BUFFER_SIZE)
        rs232_buffer_rx_ptr_wr = 0;

    }
  }

  return bytes_read;
}

int rs232_write(int rs232_device)
{
  int bytes_sent = 0;
  int length_to_write = 0;

  if(rs232_device > 0)
  {
    if(rs232_buffer_tx_empty)
      return 0;

    // I want to send as many bytes as possible, but I have to manage the
    // circular buffer. So I can send only the data before restart the
    // buffer.
    if(rs232_buffer_tx_ptr_rd < rs232_buffer_tx_ptr_wr)
      length_to_write = (rs232_buffer_tx_ptr_wr - rs232_buffer_tx_ptr_rd);
    else
      length_to_write = (RS232_BUFFER_SIZE - rs232_buffer_tx_ptr_rd);

    if(length_to_write > RS232_MAX_TX_LENGTH)
      length_to_write = RS232_MAX_TX_LENGTH;

    bytes_sent = write(rs232_device, &rs232_buffer_tx[rs232_buffer_tx_ptr_rd], length_to_write);

    if(bytes_sent > 0)
    {
      //printf("bytes_sent: %i\n", bytes_sent);
      /*for(i = 0; i < bytes_sent; i++)
        printf("[%x]", rs232_buffer_tx[rs232_buffer_tx_ptr_rd + i]);
      printf("\n");*/ 
      rs232_buffer_tx_full = 0;
      rs232_buffer_tx_data_count -= bytes_sent;
      rs232_buffer_tx_ptr_rd += bytes_sent;

      if(rs232_buffer_tx_ptr_rd == RS232_BUFFER_SIZE)
        rs232_buffer_tx_ptr_rd = 0;
      else if(rs232_buffer_tx_ptr_rd > RS232_BUFFER_SIZE)
        printf("Circular buffer critical error\n");
    }
  }

  if(rs232_buffer_tx_data_count == 0)
    rs232_buffer_tx_empty = 1;

  return bytes_sent;
}

int redirect_stdof_message(int socket_can, int bytes_read, unsigned char *message)
{
  static struct can_frame can_frame_to_write;
  static unsigned char rs232_state = 0;
  static unsigned char rs232_data_count = 0;
  
  unsigned char rs232_data_read = 0;
  int length_to_write = 0;
  int bytes_sent;

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

        can_frame_to_write.can_id = (((__u32)message[rs232_data_read] & 0x0007) << 8);
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
          bytes_sent = write(socket_can, &can_frame_to_write, sizeof(can_frame_to_write));

          if(bytes_sent == -1)
            return -1;

          if(bytes_sent != sizeof(can_frame_to_write))
            printf("Error on write: bytes sent %i\n", bytes_sent);
          
          /*if(bytes_sent == sizeof(can_frame_to_write))
          {
            //count += 1;
            count += length_to_write;
            printf("[%s] [%x] [%x]", "Write", can_frame_to_write.can_id, can_frame_to_write.can_dlc);

            for(i = 0; i < can_frame_to_write.can_dlc; i++)
              printf(" [%x]", can_frame_to_write.data[i]);

            printf("%i", count);
            printf("\n");
            fflush(stdout);
          }*/
        }
        break;

      default:
        rs232_state = 0;
        rs232_data_count = 0;
        break;
    }  // end switch
  } // end while

  return 0;
}

int can_message_handle(struct can_frame *frame, int rs232_device)
{
  int bytes_sent;
  unsigned char rs232_message[11];

  if(frame->can_id & CAN_RTR_FLAG)
    printf("Request flag found!\n");
  
  switch(frame->can_id)
  {  
    default:
      /*printf("[%s] [%x] [%x] ", "Read", frame->can_id, frame->can_dlc);
	  
      for(i = 0; i < frame->can_dlc; i++)
      {
        count += 1;
        printf("[%x] ", frame->data[i]);
      }

      printf("%i\n", count);
      */

      rs232_message[0] = 0x24;
      rs232_message[1] = (frame->can_dlc << 3) | (frame->can_id >> 8);
      rs232_message[2] = frame->can_id;
      memcpy(&rs232_message[3], frame->data, frame->can_dlc);

      bytes_sent = rs232_load_tx(rs232_message, (frame->can_dlc + 3));

      return bytes_sent;

      break;
  }  // end switch
}
