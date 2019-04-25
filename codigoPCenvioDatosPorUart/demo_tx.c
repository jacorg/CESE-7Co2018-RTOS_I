
/**************************************************

file: demo_tx.c
purpose: simple demo that transmits characters to
the serial port and print them on the screen,
exit the program by pressing Ctrl-C

compile with the command: gcc demo_tx.c rs232.c -Wall -Wextra -o2 -o test_tx

**************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "rs232.h"



int main()
{
  int i=0,n,
      cport_nr=17,        /* /dev/ttyS0 (COM1 on windows) */
      bdrate=115200;       /* 9600 baud */

  char mode[]={'8','N','1',0},
       str[2][512];

  unsigned char buf[4096];

  strcpy(str[0], "The quick brown fox jumped over the lazy grey dog.\n");

  strcpy(str[1], "Happy serial programming!\n");

  if(RS232_OpenComport(cport_nr, bdrate, mode))
  {
    printf("Can not open comport\n");
    return(0);
  }

  while(1)
  {
    //RS232_cputs(cport_nr, str[i]);
    //printf("sent: %s\n", str[i]);

    RS232_cputs(cport_nr, "C");
    

    n = RS232_PollComport(cport_nr, buf, 4095);
    if(n > 0)
      buf[n] = 0;   /* always put a "null" at the end of a string! */

    printf("%s\n",buf);
    usleep(1000000);  /* sleep for 1 Second */


    i++;

    i %= 2;
  }

  return(0);
}

