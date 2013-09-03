#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <net/if.h>

// PF stands for Protocol Family
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

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
