/**********************************************************
 Header files
 **********************************************************/

#include "stdio.h"
#include "main.h"
#include "system.h"
#include "uart.h"
#include <math.h>
#include "hardware.h"
#include "irq.h"
#include "i2c.h"
#include "gpsmath.h"
#include "adc.h"
#include "uart.h"
#include "uart1.h"
#include "ssp.h"
#include "LL_HL_comm.h"
#include "sdk.h"
#include <unistd.h>

/* *********************************************************
 Function declarations
 ********************************************************* */

//void Initialize(void);
//void feed(void);

/**********************************************************
 Global Variables
 **********************************************************/
struct HL_STATUS HL_Status;
struct IMU_RAWDATA IMU_RawData;
volatile unsigned int int_cnt = 0, cnt = 0, mainloop_cnt = 0;
volatile unsigned char mainloop_trigger = 0;
volatile unsigned int GPS_timeout = 0;
volatile char SYSTEM_initialized=0; //new

//extern unsigned char data_requested;
//extern int ZeroDepth;

volatile unsigned int trigger_cnt = 0;
unsigned int logs_per_second = 0, total_logs_per_second = 0;

unsigned char packets = 0x00;
unsigned char packetsTemp;
unsigned int uart_cnt;
unsigned char DataOutputsPerSecond = 20;

struct IMU_CALCDATA IMU_CalcData, IMU_CalcData_tmp;
struct GPS_TIME GPS_Time;
struct SYSTEM_PERMANENT_DATA SYSTEM_Permanent_Data;

float g_imu_gravity; // as reported in LL units (not exactly mg)

void timer0ISR(void) __irq
{
  T0IR = 0x01; //Clear the timer 0 interrupt
  IENABLE;
  trigger_cnt++;
  if (trigger_cnt == ControllerCyclesPerSecond)
  {
    trigger_cnt = 0;
    HL_Status.up_time++;
    HL_Status.cpu_load = mainloop_cnt;

    mainloop_cnt = 0;
  }

  if (mainloop_trigger < 10)
    mainloop_trigger++;
  g_timestamp += ControllerCyclesPerSecond;

  IDISABLE;
  VICVectAddr = 0; // Acknowledge Interrupt
}

void timer1ISR(void) __irq
{
  T1IR = 0x01; //Clear the timer 1 interrupt
  IENABLE;

  IDISABLE;
  VICVectAddr = 0; // Acknowledge Interrupt
}

/**********************************************************
 MAIN
 **********************************************************/
int main(void)
{
  static int vbat1, vbat2;
  int vbat;
  static int bat_cnt = 0, bat_warning = 1000;
  static char bat_warning_enabled = 1;

  IDISABLE;

  init();
  LL_write_init();

  HL_Status.up_time = 0;

  printf("\n\nProgramm is running ... \n");
  printf("Processor Clock Frequency: %d Hz\n", processorClockFrequency());
  printf("Peripheral Clock Frequency: %d Hz\n", peripheralClockFrequency());

  IENABLE;

  packetsTemp = packets;

  LED(1, ON);

  sdkInit();

  beeper(OFF);
  wait(5000000);
  calibrate();
  //g_imu_gravity = 1022.0; // TODO it this correct??
  while (1)
  {
    if (mainloop_trigger > 0)
    {
      mainloop_cnt++;
      if (++bat_cnt == 100)
        bat_cnt = 0;

      //battery monitoring
      vbat1 = (vbat1 * 29 + (ADC0Read(VOLTAGE_1) * 9872 / 579)) / 30; //voltage in mV //*9872/579

      HL_Status.battery_voltage_1 = vbat1;
      HL_Status.battery_voltage_2 = vbat2;

      vbat = vbat1;

      if (vbat < BATTERY_WARNING_VOLTAGE) //decide if it's really an empty battery
      {
        if (bat_warning < ControllerCyclesPerSecond * 2)
          bat_warning++;
        else
          bat_warning_enabled = 1;
      }
      else
      {
        if (bat_warning > 10)
          bat_warning -= 5;
        else
        {
          bat_warning_enabled = 0;
          beeper(OFF);
        }
      }
      if (bat_warning_enabled)
      {
        if (bat_cnt > ((vbat - 9000) / BAT_DIV))
          beeper(ON);
        else
          beeper(OFF);
      }

      if (mainloop_trigger)
        mainloop_trigger--;
      mainloop();
    }
  }
  return 0;
}


void mainloop(void)
{
  SDK_mainloop();

  HL2LL_write_cycle(); //write data to transmit buffer for immediate transfer to LL processor
}

void calibrate()
{
  double sum = 0.0;
  unsigned int i;
  for (i = 0; i < 2000; ++i)
  {
    wait(1200); // wait 1 ms
    HL2LL_write_cycle();
    double ax = RO_ALL_Data.acc_x;
    double ay = RO_ALL_Data.acc_y;
    double az = RO_ALL_Data.acc_z;

    double a = sqrt(ax*ax + ay*ay + az*az);
    sum +=a;
  }

  g_imu_gravity  = sum / 2000.0;
}

