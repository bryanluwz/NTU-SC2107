// RSLK Self Test via UART

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

#include "msp.h"
#include <stdint.h>
#include <string.h>
#include "..\inc\UART0.h"
#include "..\inc\EUSCIA0.h"
#include "..\inc\FIFO0.h"
#include "..\inc\Clock.h"
//#include "..\inc\SysTick.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\TimerA1.h"
//#include "..\inc\Bump.h"
#include "..\inc\BumpInt.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Motor.h"
#include "../inc/IRDistance.h"
#include "../inc/ADC14.h"
#include "../inc/LPF.h"
#include "..\inc\Reflectance.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/Tachometer.h"

#define P2_4 (*((volatile uint8_t *)(0x42098070)))
#define P2_3 (*((volatile uint8_t *)(0x4209806C)))
#define P2_2 (*((volatile uint8_t *)(0x42098068)))
#define P2_1 (*((volatile uint8_t *)(0x42098064)))
#define P2_0 (*((volatile uint8_t *)(0x42098060)))


void RSLK_Reset(void){
    DisableInterrupts();

    LaunchPad_Init();
    //Initialise modules used e.g. Reflectance Sensor, Bump Switch, Motor, Tachometer etc
    // ... ...

    EnableInterrupts();
}

void TimedPause(uint32_t time){
  Clock_Delay1ms(time);          // run for a while and stop
  Motor_Stop();
}

// IR Sensors thing
volatile uint32_t ADCvalue;
volatile uint32_t ADCflag;
volatile uint32_t nr,nc,nl;

void SensorRead_ISR(void){  // runs at 2000 Hz
    uint32_t raw17,raw12,raw16;
    P1OUT ^= 0x01;         // profile
    P1OUT ^= 0x01;         // profile
    ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
    nr = LPF_Calc(raw17);  // right is channel 17 P9.0
    nc = LPF_Calc2(raw12);  // center is channel 12, P4.1
    nl = LPF_Calc3(raw16);  // left is channel 16, P9.1
    ADCflag = 1;           // semaphore
    P1OUT ^= 0x01;         // profile
}

// Bumper thing
uint8_t bump_data;
int bump0 = 0, bump1 = 0, bump2 = 0, bump3 = 0, bump4 = 0, bump5 = 0;
char bumperBuf[30];

// Bumper thing to exit while loop
volatile uint8_t CollisionData, CollisionFlag;
void HandleCollision(uint8_t bumpSensor){
   Motor_Stop();
   CollisionData = bumpSensor;
   CollisionFlag = 1;
   P4->IFG &= ~0xED;                  // clear interrupt flags (reduce possibility of extra interrupt)

}

// Tachometer thing
uint16_t Period0;              // (1/SMCLK) units = 83.3 ns units
uint16_t First0=0;             // Timer A3 first edge, P10.4
uint32_t Done0=0;              // set each rising

uint16_t Period2;              // (1/SMCLK) units = 83.3 ns units
uint16_t First2=0;             // Timer A3 first edge, P8.2
uint32_t Done2=0;              // set each rising
uint32_t LeftRPM = 0, RightRPM = 0;

// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure0(uint16_t time){
  Period0 = (time - First0)&0xFFFF; // 16 bits, 83.3 ns resolution
  First0 = time;                    // setup for next
  Done0++;
}

// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure2(uint16_t time){
  Period2 = (time - First2)&0xFFFF; // 16 bits, 83.3 ns resolution
  First2 = time;                    // setup for next
  Done2++;
}

// Other useless things
const char *byte_to_binary
(
    int x
)
{
    static char b[9];
    b[0] = '\0';

    int z;
    for (z = 128; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}


// RSLK Self-Test
// Sample program of how the text based menu can be designed.
// Only one entry (RSLK_Reset) is coded in the switch case. Fill up with other menu entries required for Lab5 assessment.
// Init function to various peripherals are commented off.  For reference only. Not the complete list.

int main(void) {
  uint32_t cmd=0xDEAD, menu=0, count = 0;

  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  //SysTick_Init(48000,2);  // set up SysTick for 1000 Hz interrupts
  Motor_Init();
  //Motor_Stop();
  LaunchPad_Init();
  BumpInt_Init(&HandleCollision);
  //Bumper_Init();
  //IRSensor_Init();
  //Tachometer_Init();
  EUSCIA0_Init();     // initialize UART
  EnableInterrupts();

  while(1){                     // Loop forever
      // write this as part of Lab 5
      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("RSLK Testing"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[0] RSLK Reset"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[1] Motor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[2] IR Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[3] Bumper Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[4] Reflectance Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[5] Tachometer Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[6] Simultaneous Bumper Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
//      EUSCIA0_OutString("[7] Obstacle Avoidance (IR)"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

      EUSCIA0_OutString("CMD: ");
      cmd=EUSCIA0_InUDec();
      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

      switch(cmd){
          case 0:
              RSLK_Reset();
              menu =1;
              cmd=0xDEAD;

              UART0_OutString("Reseting... :P\r\n");

              break;

          case 1:
              // Motor Test
              UART0_OutString("Starting motor test... :D\r\n");

              UART0_OutString("\033[A\33[2K\rMoving forward... :>\n");
              Motor_Forward(3000,3000);
              TimedPause(1000);
              UART0_OutString("\033[A\33[2K\rMoving backward... :>\n");
              Motor_Backward(3000,3000);
              TimedPause(1000);
              UART0_OutString("\033[A\33[2K\rTurning left... :>\n");
              Motor_Left(3000,3000);
              TimedPause(1000);
              UART0_OutString("\033[A\33[2K\rTurning right... :>\n");
              Motor_Right(3000,3000);
              TimedPause(1000);
              UART0_OutString("\033[A\33[2K\rMoving right wheel... :>\n");
              Motor_Forward(3000,0);
              TimedPause(1000);
              UART0_OutString("\033[A\33[2K\rMoving left wheel... :>\n");
              Motor_Forward(0,3000);
              TimedPause(1000);
              Motor_Stop();
              break;

          case 2:
              // IR Sensor test
              CollisionFlag = 0;

              char irBuf[100];
              uint32_t raw17,raw12,raw16;
              uint32_t s, n;

              ADCflag = 0;
              s = 256; // replace with your choice

              ADC0_InitSWTriggerCh17_12_16();   // initialize channels 17,12,16

              ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample

              LPF_Init(raw17,s);     // P9.0/channel 17
              LPF_Init2(raw12,s);     // P4.1/channel 12
              LPF_Init3(raw16,s);     // P9.1/channel 16

              UART0_Init();          // initialize UART0 115,200 baud rate

              TimerA1_Init(&SensorRead_ISR,250);    // 2000 Hz sampling
              UART0_OutString("Testing IR sensors...\r\n");
              EnableInterrupts();

              while(1){
                  for(n=0; n<250; n++){
                      while (ADCflag == 0);
                      ADCflag = 0; // show every nth point
                      }
                  if (CollisionFlag) {
                      EUSCIA0_OutString("\r\nBoop! :O Bumper Interrupt Detected!\r\n");
                      CollisionFlag = 0;
                      break;
                  }
                    sprintf(irBuf, "\033[A\33[2K\r%5d mm, %5d mm, %5d mm\n", LeftConvert(nl), CenterConvert(nc), RightConvert(nr));
                    UART0_OutString(irBuf);
              }

              break;

          case 3:
              // Bumpers
              CollisionFlag = 0;

        	  count = 0;

              UART0_OutString("Checking each individual bumpers\r\n");
              UART0_OutString("Press the bumpers please. >o<\r\n\r\n");

              while (!bump0 || !bump1 || !bump2 || !bump3 || !bump4 || !bump5) {
                  bump_data = Bump_Read();
//                  if (count % 200 == 0) {
//                      UART0_OutUHex(bump_data);UART0_OutString(" \r\n");
//                  }

                  // Check whether each bumper is pressed, then set bump value to 1
                  if ((((bump_data >> 0) & 0x1)) == 0 && bump0 == 0) {
                      bump0 = 1;
                      UART0_OutString("Bumper 0 pressed \r\n");
                  }

                  if ((((bump_data >> 1) & 0x1)) == 0 && bump1 == 0) {
                    bump1 = 1;
                    UART0_OutString("Bumper 1 pressed \r\n");
                    }

                  if ((((bump_data >> 2) & 0x1)) == 0 && bump2 == 0) {
                    bump2 = 1;
                    UART0_OutString("Bumper 2 pressed \r\n");
                }

                  if ((((bump_data >> 3) & 0x1)) == 0 && bump3 == 0) {
                        bump3 = 1;
                        UART0_OutString("Bumper 3 pressed \r\n");
                    }

                  if ((((bump_data >> 4) & 0x1)) == 0 && bump4 == 0) {
                        bump4 = 1;
                        UART0_OutString("Bumper 4 pressed \r\n");
                    }

                  if ((((bump_data >> 5) & 0x1)) == 0 && bump5 == 0) {
                        bump5 = 1;
                        UART0_OutString("Bumper 5 pressed \r\n");
                        }

                  count++;
              }

              UART0_OutString("All bumpers pressed :3 \r\n");

              break;

          case 4:
              // Reflectance thing
              CollisionFlag = 0;

              uint8_t Data; // QTR-8RC
              int32_t Position; // 332 is right, and -332 is left of center
              count = 0;\
              char refBuf[20] = "";

              while(1){

                  if (CollisionFlag) {
                      EUSCIA0_OutString("\r\nBoop! :O Bumper Interrupt Detected!\r\n");
                      CollisionFlag = 0;
                      break;
                  }
                  Data = Reflectance_Read(2000);
                  Position = Reflectance_Position(Data);

                  if (count % 10 == 0) {
                        sprintf(refBuf, "\033[A\33[2K\r%s, %d mm\n", byte_to_binary(Data), Position / 10);
                        UART0_OutString(refBuf);
                  }

                  Clock_Delay1ms(10);
                  count ++;
                }
              break;

          case 5:
              CollisionFlag = 0;
              EnableInterrupts();

              // Tachometer test
              count = 0;
              char tachBuf[69];

              TimerA3Capture_Init(&PeriodMeasure0,&PeriodMeasure2);
              TimedPause(500);
              Motor_Forward(1500,1500);
              EnableInterrupts();
              while(1){
                    WaitForInterrupt();
                    count++;
                    if(count%100==0){
                        LeftRPM = ((60 * 1000 * 1000 ) / (360 * 83.3 * Period0 / 1000));
                        RightRPM = ((60 * 1000 * 1000 ) / (360 * 83.3 * Period2 / 1000));
                        sprintf(tachBuf, "\033[A\33[2K\rPeriod0 = %5d, Right RPM = %2d, Period2 = %5d, Left RPM = %2d\n", Period0, RightRPM, Period2, LeftRPM);
                        UART0_OutString(tachBuf);
                    }
                    if (CollisionFlag) {
                        EUSCIA0_OutString("\r\nBoop! :O Bumper Interrupt Detected!\r\n");
                        CollisionFlag = 0;
                        break;
                    }
              }
              break;

          case 6:
        	  // Bumpers
			  count = 0;

				UART0_OutString("Checking for simultaneous bumpers press\r\n");
				UART0_OutString("Press the bumpers please. >o<\r\n\r\n");

				while (1) {

					int pressed = 0;
					strcpy(bumperBuf, "\033[A\33[2K\rBumper ");
					 bump_data = Bump_Read();
					 if ((((bump_data >> 0) & 0x1)) == 0) {
						 strcat(bumperBuf, "0 ");
						 pressed = 1;
					 }
					 if ((((bump_data >> 1) & 0x1)) == 0) {
						 strcat(bumperBuf, "1 ");
						 pressed = 1;
					 }
					 if ((((bump_data >> 2) & 0x1)) == 0) {
						 strcat(bumperBuf, "2 ");
						 pressed = 1;
					 }
					 if ((((bump_data >> 3) & 0x1)) == 0) {
						 strcat(bumperBuf, "3 ");
						 pressed = 1;
					 }
					 if ((((bump_data >> 4) & 0x1)) == 0) {
						 strcat(bumperBuf, "4 ");
						 pressed = 1;
					 }
					 if ((((bump_data >> 5) & 0x1)) == 0) {
						 strcat(bumperBuf, "5 ");
						 pressed = 1;
					 }

					 strcat(bumperBuf, "pressed.\n");

					 if (!pressed) {
						 strcpy(bumperBuf, "\033[A\33[2K\rNo bumpers pressed\n ");
					 }
					 count++;
					 if (count % 2000 == 0) {
						 UART0_OutString(bumperBuf);
						 if ((bump_data & 0b111111) == 0) {
							 UART0_OutString("\r\nAll bumpers pressed :3, goodbye!\r\n");
							 break;
						 }
					 }

				}


				break;

//          case 7:
//              // IR Sensor test
//            CollisionFlag = 0;
//
//            char temp7Buf[100];
//
//            ADCflag = 0;
//            s = 256; // replace with your choice
//
//            ADC0_InitSWTriggerCh17_12_16();   // initialize channels 17,12,16
//
//            ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
//
//            LPF_Init(raw17,s);     // P9.0/channel 17
//            LPF_Init2(raw12,s);     // P4.1/channel 12
//            LPF_Init3(raw16,s);     // P9.1/channel 16
//
//            UART0_Init();          // initialize UART0 115,200 baud rate
//
//            TimerA1_Init(&SensorRead_ISR,250);    // 2000 Hz sampling
//            UART0_OutString("Testing IR sensors...\r\n");
//            EnableInterrupts();
//
//            while(1){
////                for(int n=0; n<250; n++){
////                    while (ADCflag == 0);
////                    ADCflag = 0; // show every nth point
////                }
//                Motor_Forward(3000,3000);
//
//                if(LeftConvert(nl)<=100){
//                    Motor_Stop();
//                    Motor_Right(3000,3000);
//                    Clock_Delay1ms(740);
//                }
//                else if(CenterConvert(nc)<=100){
//                    Motor_Stop();
//                    Motor_Backward(3000,3000);
//                    Clock_Delay1ms(500);
//                    Motor_Right(3000,3000);
//                    Clock_Delay1ms(1400);
//                }
//                else if(RightConvert(nr)<=100){
//                    Motor_Stop();
//                    Motor_Left(3000,3000);
//                    Clock_Delay1ms(740);
//                }
//                if (CollisionFlag) {
//                    EUSCIA0_OutString("\r\nBoop! :O Bumper Interrupt Detected!\r\n");
//                    CollisionFlag = 0;
//                    Motor_Stop();
//                    break;
//                }
//                  sprintf(temp7Buf, "\033[A\33[2K\r%5d mm, %5d mm, %5d mm\n", LeftConvert(nl), CenterConvert(nc), RightConvert(nr));
//                  UART0_OutString(temp7Buf);
//                  Motor_Stop();
//            }
//
//            break;

          default:
              menu=1;
              break;
      }

      if(!menu)Clock_Delay1ms(1000);
      else{
          menu=0;
      }

      // ....
      // ....
  }
}

#if 0
//Sample program for using teh UART related functions.
int Program5_4(void){
//int main(void){
    // demonstrates features of the EUSCIA0 driver
  char ch;
  char string[20];
  uint32_t n;
  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  EUSCIA0_Init();     // initialize UART
  EnableInterrupts();
  EUSCIA0_OutString("\nLab 5 Test program for EUSCIA0 driver\n\rEUSCIA0_OutChar examples\n");
  for(ch='A'; ch<='Z'; ch=ch+1){// print the uppercase alphabet
     EUSCIA0_OutChar(ch);
  }
  EUSCIA0_OutChar(LF);
  for(ch='a'; ch<='z'; ch=ch+1){// print the lowercase alphabet
    EUSCIA0_OutChar(ch);
  }
  while(1){
    EUSCIA0_OutString("\n\rInString: ");
    EUSCIA0_InString(string,19); // user enters a string
    EUSCIA0_OutString(" OutString="); EUSCIA0_OutString(string); EUSCIA0_OutChar(LF);

    EUSCIA0_OutString("InUDec: ");   n=EUSCIA0_InUDec();
    EUSCIA0_OutString(" OutUDec=");  EUSCIA0_OutUDec(n); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString(" OutUFix1="); EUSCIA0_OutUFix1(n); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString(" OutUFix2="); EUSCIA0_OutUFix2(n); EUSCIA0_OutChar(LF);

    EUSCIA0_OutString("InUHex: ");   n=EUSCIA0_InUHex();
    EUSCIA0_OutString(" OutUHex=");  EUSCIA0_OutUHex(n); EUSCIA0_OutChar(LF);
  }
}
#endif

