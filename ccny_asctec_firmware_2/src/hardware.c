#include "hardware.h"

void LED(unsigned char nr, unsigned char onoff) //set or reset LED 0..3
{
  if (nr>=2)
  	return;
  if(onoff == OFF)
  {
    IOSET1 = (1<<(24+nr));
  }
  else
  {
    IOCLR1 = (1<<(24+nr));
  }
}

void beeper(unsigned char offon)
{
  if (offon) //beeper on
  {
    IOSET1 = (1 << 17);
  }
  else
  {
    IOCLR1 = (1 << 17);
  }
}
