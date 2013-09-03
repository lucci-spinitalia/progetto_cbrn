#ifndef __RS232_H_
#define __RS232_H_

int com_open(char *device_name, __u32 rate, char parity, int data_bits, int stop_bits);

#endif
