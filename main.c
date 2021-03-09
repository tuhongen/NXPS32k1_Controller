/*
   Copyright (c) 2015 - 2016 , Freescale Semiconductor, Inc.
   Copyright 2016-2017 NXP
   All rights reserved.

   THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
   IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
   THE POSSIBILITY OF SUCH DAMAGE.
*/
/* ###################################################################
**     Filename    : main.c
**     Project     : lin_master_s32k144
**     Processor   : S32K144_100
**     Version     : Driver 01.00
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-09-22, 10:58, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.00
** @brief
**         Main module.
**         This module contains user's application code.
*/
/*!
**  @addtogroup main_module main module documentation
**  @{
*/
/* MODULE main */
#include "clocks_and_modes.h"
#include "LPUART.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "clockMan1.h"
#include "pin_mux.h"
#include "dmaController1.h"
#include "lpTmr1.h"
//#include "lin_common_api.h"

#include "FlexCAN.h"


#if CPU_INIT_CONFIG
#include "Init_Config.h"
#endif
volatile int exit_code = 0;
/* User includes (#include below this line is not maintained by Processor Expert) */
/* This example is setup to work by default with EVB. To use it with other boards
   please comment the following line
*/
#define EVB

#ifdef EVB
#define PORT_LED0_INDEX           (0u)
#define PORT_LED1_INDEX           (15u)
#define PORT_LED2_INDEX           (16u)
#define PORT_LED0_MASK            (0x1u << PORT_LED0_INDEX)
#define PORT_LED1_MASK            (0x1u << PORT_LED1_INDEX)
#define PORT_LED2_MASK            (0x1u << PORT_LED2_INDEX)
#define LED_GPIO_PORT             (PTD)
#define PORT_BTN0_INDEX           (12u)
#define PORT_BTN1_INDEX           (13u)
#define PORT_BTN0_MASK            (0x1u << PORT_BTN0_INDEX)
#define PORT_BTN1_MASK            (0x1u << PORT_BTN1_INDEX)
#define BTN_GPIO_PORT             (PTC)
#define BTN_PORT_NAME             (PORTC)
#define BTN_PORT_IRQn             (PORTC_IRQn)
#define USE_LIN_XCVR              (0)  //htu turn off LIN
#define LIN_XCVR_ENABLE_PIN       (9UL)
#define LIN_XCVR_ENABLE_MASK      (0x1u << LIN_XCVR_ENABLE_PIN)
#define LIN_XCVR_ENABLE_GPIO_PORT (PTE)
#else
#define PORT_LED0_INDEX           (0u)
#define PORT_LED1_INDEX           (1u)
#define PORT_LED2_INDEX           (2u)
#define PORT_LED0_MASK            (0x1u << PORT_LED0_INDEX)
#define PORT_LED1_MASK            (0x1u << PORT_LED1_INDEX)
#define PORT_LED2_MASK            (0x1u << PORT_LED2_INDEX)
#define LED_GPIO_PORT             (PTC)
#define PORT_BTN0_INDEX           (12u)
#define PORT_BTN1_INDEX           (13u)
#define PORT_BTN0_MASK            (0x1u << PORT_BTN0_INDEX)
#define PORT_BTN1_MASK            (0x1u << PORT_BTN1_INDEX)
#define BTN_GPIO_PORT             (PTC)
#define BTN_PORT_NAME             (PORTC)
#define BTN_PORT_IRQn             (PORTC_IRQn)
#endif

/* (CLK (MHz)* timer period (us) / Prescaler) */
#define TIMER_COMPARE_VAL (uint16_t)(2000U)
#define TIMER_TICKS_1US   (uint16_t)(4U)

#define MOTOR_SELECTION_INCREASE (1u)
#define MOTOR_SELECTION_DECREASE (2u)
#define MOTOR_SELECTION_STOP     (3u)

#define MOTOR1_OVER_TEMP   (200u)
#define MOTOR1_MAX_TEMP    (100u)
#define MOTOR1_MIN_TEMP    (30u)

#define MSG_BUF_SIZE  4

uint32_t  RxCODE;              /* Received message buffer code */
uint32_t  RxID;                /* Received message ID  14575-5427 */
uint32_t  RxLENGTH, AngleLimitP = 0x38EF, AngleLimitN = 0x1533;      /* Recieved message number of data bytes */
int32_t  RxDATA[2];           /* Received message data (2 words) */
int32_t  Error;
float speed, area, delta_t = 0.02, ControlAngle;
float decel_limit = 2500, accel_limit = 2500; speed_limit = 2500;
uint32_t  RxTIMESTAMP;         /* Received message time */
uint32_t dummy, INT_Control_Angle; //CONVERT ControlAngle to uint32
int16_t CurrentSteeringAngle, TargetSteeringAngle = 0x2779, AngleDir, AngleBuff;
int16_t preTSA = 0x2779 , deltaTSA = 0, rateTSA_acc = 10, rateTSA_dec = 10, rateTSA_spd = 10;
uint8_t j, SteeringControlStatus, EnableTransition = 0;
uint32_t counter = 0;
uint16_t timerOverflowInterruptCount = 0U;
int Motor1_temp;
uint8_t Node;
uint8_t TxbufferNo = 0, BufLimit = 15, RxBufferNo = 15; //initial CAN rx buffer
int16_t to16Buf;
bool Node1Flag, Node2Flag, Node3Flag, Node4Flag;
long double Testing;
bool SteerByWire = 0;

/*!
   @brief LPTMR Interrupt Service Routine
   The ISR will call LIN timeout service every 500us
   and will provide the required tick for LIN (every 5 ms)
*/
void LPTMR_ISR(void)
{
  /* Static variable, used to count if the timeout has passed to
     provide the LIN scheme tick.
  */
  static uint32_t interruptCount = 0UL;
  static uint32_t interruptCount2 = 0UL;
  static int OneTimeCounter = 1;
  /* Timer Interrupt Handler */
  //    lin_lld_timeout_service(LI0);
  if (OneTimeCounter)
  {
    //l_sch_tick(LI0);
    interruptCount2++;
    if (interruptCount2 == 1000UL)
    {
      OneTimeCounter = 0;
      //       l_sch_set(LI0, LI0_NormalTable, 0u);
    }

  }
  /* If 5 ms have passed, provide the required tick */
  if (++interruptCount == 10UL)
  {
    //     l_sch_tick(LI0);
    interruptCount = 0UL;
  }

  /* Increment overflow count */
  timerOverflowInterruptCount++;
  /* Clear compare flag */
  LPTMR_DRV_ClearCompareFlag(INST_LPTMR1);
}

/*!
   @brief Callback function to get time interval in nano seconds
   @param[out] ns - number of nanoseconds passed since the last call of the function
   @return dummy value
*/
uint32_t timerGetTimeIntervalCallback0(uint32_t *ns)
{
  static uint32_t previousCountValue = 0UL;
  uint32_t counterValue;

  counterValue = LPTMR_DRV_GetCounterValueByCount(INST_LPTMR1);
  *ns = ((uint32_t)(counterValue + timerOverflowInterruptCount * TIMER_COMPARE_VAL - previousCountValue)) * 1000 / TIMER_TICKS_1US;
  timerOverflowInterruptCount = 0UL;
  previousCountValue = counterValue;
  return 0UL;
}

/*!
   @brief Interrupt Service Routine for buttons
   Depending on which buttons were pressed LIN scheme will be
   set to sleep mode or normal mode.
*/
void BTNPORT_IRQHandler(void)
{
  /* If SW2/Button 1 is pressed */
  if (PINS_DRV_GetPortIntFlag(BTN_PORT_NAME) & (1 << PORT_BTN0_INDEX))
  {
    PINS_DRV_ClearPinIntFlagCmd(BTN_PORT_NAME, PORT_BTN0_INDEX);
    //        l_sch_set(LI0, LI0_Table1, 0u);
    //        LPUART1_transmit_string("SAT Command");
    //        FLEXCAN0_transmit_Data(0x11223344,0x55667788,0x3A8);
    FLEXCAN1_transmit_Data(0x11223344, 0x55667788, 0x3A8);
    //       FLEXCAN2_transmit_Data(0x11223344,0x55667788,0x3A8);
    SteerByWire = !SteerByWire;

  }

  /* If SW3/Button 2 is pressed */
  if (PINS_DRV_GetPortIntFlag(BTN_PORT_NAME) & (1 << PORT_BTN1_INDEX))
  {
    PINS_DRV_ClearPinIntFlagCmd(BTN_PORT_NAME, PORT_BTN1_INDEX);
    /* Toggle RED Led */
    PINS_DRV_TogglePins(LED_GPIO_PORT, PORT_LED1_MASK);
    //      PINS_DRV_SetPins(LED_GPIO_PORT, PORT_LED2_MASK);
    //      PINS_DRV_SetPins(LED_GPIO_PORT, PORT_LED0_MASK);
    TargetSteeringAngle = 0x2716;
    //       FLEXCAN2_transmit_Data(0x11223344,0x55667788,0x3A8);
    /* Switch to normal table */
    //    l_ifc_wake_up(LI0);
    //        l_sch_set(LI0, LI0_NormalTable, 0u);
    //       LPUART1_transmit_string("SAT Status");
  }
}


void CAN0_IRQHandler(void)
{

  while ((CAN0->RAMn[ 4 * MSG_BUF_SIZE + 0] & 0x01000000) >> 24)  {} // htu makesure code field is not busy
  //  RxCODE   = (CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] & 0x07000000) >> 24;  /* Read CODE field */
  RxID     = (CAN0->RAMn[ 4 * MSG_BUF_SIZE + 1] & CAN_WMBn_ID_ID_MASK)  >> 18 ; // shift 18 bit to get 11 standard ID
  RxLENGTH = (CAN0->RAMn[ 4 * MSG_BUF_SIZE + 0] & CAN_WMBn_CS_DLC_MASK) >> CAN_WMBn_CS_DLC_SHIFT;
  for (j = 0; j < 2; j++) { /* Read two words of data (8 bytes) */
    RxDATA[j] = CAN0->RAMn[ 4 * MSG_BUF_SIZE + 2 + j];
  }
  RxTIMESTAMP = (CAN0->RAMn[ 4 * MSG_BUF_SIZE + 0] & 0x000FFFF);
  dummy = CAN0->TIMER;             /* Read TIMER to unlock message buffers */
  CAN0->IFLAG1 = 0x00000010;       /* Clear CAN 0 MB 4 flag without clearing others*/


  if (RxID == 0x101)//mkC
//  if (RxID == 0x112)//mkZ
  { if ((SteeringControlStatus == 0x01) || (SteeringControlStatus == 0x02)) //steering module open and under control
    {


	 // decel_limit= (RxDATA[1]>>24 & 0x000000FF)*40;
	 // accel_limit= (RxDATA[1]>>16 & 0x000000FF)*40;
	  //speed_limit= (RxDATA[1]>>8  & 0x000000FF)*40;

	  // counter=RxDATA[0];
      to16Buf = RxDATA[0] & 0x0000FFFF;
      TargetSteeringAngle = (to16Buf / 10 + 1000) * 10;
      if (TargetSteeringAngle > AngleLimitP) {
        TargetSteeringAngle = AngleLimitP;
      }
      else if (TargetSteeringAngle < AngleLimitN) {
        TargetSteeringAngle = AngleLimitN;
      }
      SteerByWire = RxDATA[0] >> 16;
      //       EnableTransition=EnableTransition<<1+SteerByWire;

      if (TargetSteeringAngle != preTSA)
      {  deltaTSA = TargetSteeringAngle-preTSA;
      // Different acc. rate for diff. delta Target
    	  if ( (copysign(deltaTSA,1)> 0)|(copysign(deltaTSA,1) <= 200)) { rateTSA_acc = 10, rateTSA_dec = 10,rateTSA_spd =10 ;}
          if ( (copysign(deltaTSA,1)> 200)|(copysign(deltaTSA,1) <= 500)) {rateTSA_acc = 20, rateTSA_dec = 15,rateTSA_spd =20;}
          if ( (copysign(deltaTSA,1)> 500)|(copysign(deltaTSA,1) <= 2000)) { rateTSA_acc = 30, rateTSA_dec = 20,rateTSA_spd =30;}
          if ( (copysign(deltaTSA,1 )>2000)|(copysign(deltaTSA,1)<=10000)) { rateTSA_acc = 40, rateTSA_dec = 20,rateTSA_spd = 40;}

	  decel_limit= (RxDATA[1]>>24 & 0x000000FF)* rateTSA_dec;
	  accel_limit= (RxDATA[1]>>16 & 0x000000FF)* rateTSA_acc;
	  speed_limit= (RxDATA[1]>>8  & 0x000000FF)* rateTSA_spd;

	  preTSA = TargetSteeringAngle;

      }


      }

    }
    else
    { // print out error message indicate Steering module not open!
    }
  }


void CAN1_IRQHandler(void)
{

  while ((CAN1->RAMn[ RxBufferNo * MSG_BUF_SIZE + 0] & 0x01000000) >> 24)  {} /* htu makesure code field is not busy*/
  //  RxCODE   = (CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] & 0x07000000) >> 24;  /* Read CODE field */
  RxID     = (CAN1->RAMn[ RxBufferNo * MSG_BUF_SIZE + 1] & CAN_WMBn_ID_ID_MASK)  >> 18 ; /* shift 18 bit to get 11 standard ID*/
  RxLENGTH = (CAN1->RAMn[ RxBufferNo * MSG_BUF_SIZE + 0] & CAN_WMBn_CS_DLC_MASK) >> CAN_WMBn_CS_DLC_SHIFT;
  for (j = 0; j < 2; j++) { /* Read two words of data (8 bytes) */
    RxDATA[j] = CAN1->RAMn[ RxBufferNo * MSG_BUF_SIZE + 2 + j];
  }
  if (RxID == 0x3A8) {
    CurrentSteeringAngle = RxDATA[0] & 0X7FFF;  /* buffer the most current Steering colum Angle*/
  }
  RxTIMESTAMP = (CAN1->RAMn[ RxBufferNo * MSG_BUF_SIZE + 0] & 0x000FFFF);
  dummy = CAN1->TIMER;             /* Read TIMER to unlock message buffers */
  CAN1->IFLAG1 = 0x00008000;       /* Clear CAN 0 MB 4 flag without clearing others*/
  /* Toggle RED Led */

  if ((RxID == 0x3A8) && SteerByWire)
  {
    EnableTransition = EnableTransition << 1 | 0x1; //Track EnableTransition
    if ((EnableTransition & 0x3) == 0x01) //Enable Initial Transition from 0 to 1
    { ControlAngle = CurrentSteeringAngle; //initialize ControlAngle based on actual steering angle
      speed=0; //initialize steering speed
    }
    Error = TargetSteeringAngle - ControlAngle;// measure error
    // four conditions
    //1.error>0 && speed>0
    //2.error>0 && speed<0
    //3.error<0 && speed>0
    //4.error<0 && speed<0
    if (Error > 0 )
    {
      if (speed >0){// condition-1 speed is positive and error is positive steering wheel is running towards target
        area = 0.5 * speed * speed / decel_limit + 10;
        if (Error <= area){ // check if steering angle over shoot with max decel
          speed = speed - decel_limit * delta_t;// it will over shoot, so slow the speed
        }else{// it will not over shoot speed up
          speed = speed + accel_limit * delta_t;
          if (speed > speed_limit){ // after speed up check if speed is over limit
            speed = speed_limit;
          }
        }
      }else{// condition-2 -  speed is neigitave but error is positive steering wheel is running away from the target
        speed = speed + 2.0* decel_limit * delta_t; // slow down the steering wheel first, may need 2* dec. for quick change direction.
        if (speed > speed_limit){ // check if steering wheel is over speed limit
          speed = speed_limit;
        }
      }
    } else {
      if (speed < 0){// condition-3 speed is neigitive and error is neigitave steering wheel is running towards target
        area = 0.5 * speed * speed / decel_limit + 10;
        if (-Error <= area){// check if sttering angle will over shoot with max decel
          speed = speed + decel_limit * delta_t;// it will over shoot so slow down
        }else{// not it will not over shoot
          speed = speed - accel_limit * delta_t;//so speed up by making the speed smaller because speed is smaller than zero
          if (-speed > speed_limit){// check if speed is over limit
            speed = -speed_limit;
          }
        }
      }else{// condition-4 speed is positive but error is neigitave steering wheel is running ccw but target is on its cw psoition
        speed = speed - 2.0*decel_limit * delta_t; // so slow down the speed first, 1.5* to quickly change direction.
        if (-speed > speed_limit){// check if speed is over the limit
          speed = - speed_limit;
        }
      }
    }


    ControlAngle = ControlAngle + speed * delta_t;
    INT_Control_Angle = ControlAngle;

    /*output directly the targetAngle*/
    //          CAN2->IFLAG1 = 0x00000001;       /* Clear CAN 0 MB 0 flag without clearing others*/
    CAN2->RAMn[ TxbufferNo * MSG_BUF_SIZE + 2] =  RxDATA[0] & 0xFFFF0000 | 0x8000 | INT_Control_Angle; // Insert Target Angle
    CAN2->RAMn[ TxbufferNo * MSG_BUF_SIZE + 3] =  RxDATA[1]; /* MB0 word 3: data word 1 */
    CAN2->RAMn[ TxbufferNo * MSG_BUF_SIZE + 1] = CAN1->RAMn[ RxBufferNo * MSG_BUF_SIZE + 1]; /* MB0 word 1: Tx msg with STD ID 0x555 */
    CAN2->RAMn[ TxbufferNo * MSG_BUF_SIZE + 0] = 0x0C400000 | 8 << CAN_WMBn_CS_DLC_SHIFT; /* MB0 word 0: */
    TxbufferNo++;
    if (TxbufferNo >= BufLimit) {
      TxbufferNo = 0; //cycle TX buffer               /* EDL,BRS,ESI=0: CANFD not used */
    }
    /* CODE=0xC: Activate msg buf to transmit */
    /* IDE=0: Standard ID */
    /* SRR=1 Tx frame (not req'd for std ID) */
    /* RTR = 0: data, not remote tx request frame*/
    /* DLC = 8 bytes */
  }
  else   //relay other messages and 3A8 without steering control.
  {
    //      CAN2->IFLAG1 = 0x00000001;       /* Clear CAN 0 MB 0 flag without clearing others*/
    CAN2->RAMn[ TxbufferNo * MSG_BUF_SIZE + 2] =  RxDATA[0]; /* MB0 word 2: data word 0 */
    CAN2->RAMn[ TxbufferNo * MSG_BUF_SIZE + 3] =  RxDATA[1]; /* MB0 word 3: data word 1 */

    CAN2->RAMn[ TxbufferNo * MSG_BUF_SIZE + 1] = CAN1->RAMn[ RxBufferNo * MSG_BUF_SIZE + 1]; /* MB0 word 1: Tx msg with STD ID 0x555 */

    CAN2->RAMn[ TxbufferNo * MSG_BUF_SIZE + 0] = 0x0C400000 | RxLENGTH << CAN_WMBn_CS_DLC_SHIFT;
    TxbufferNo++;
    if (TxbufferNo >= BufLimit) {
      TxbufferNo = 0; //cycle TX buffer
    }
    //  CAN2->RAMn[ 0*MSG_BUF_SIZE + 0] = CAN1->RAMn[ 0*MSG_BUF_SIZE + 0]&0x00FF0000 |0x0C400000 ; /* MB0 word 0: */
    /* EDL,BRS,ESI=0: CANFD not used */
    /* CODE=0xC: Activate msg buf to transmit */
    /* IDE=0: Standard ID */
    /* SRR=1 Tx frame (not req'd for std ID) */
    /* RTR = 0: data, not remote tx request frame*/
    /* DLC = 8 bytes */

    if (RxID == 0x3A8)//Relay 3A8 to CAN0

    { EnableTransition = EnableTransition << 1; //
      CAN0->RAMn[ 0 * MSG_BUF_SIZE + 2] =  RxDATA[0]; //Increamental Angle output
      CAN0->RAMn[ 0 * MSG_BUF_SIZE + 3] =  RxDATA[1]; /* MB0 word 3: data word 1 */
      CAN0->RAMn[ 0 * MSG_BUF_SIZE + 1] = CAN1->RAMn[ RxBufferNo * MSG_BUF_SIZE + 1]; /* MB0 word 1: Tx msg with STD ID 0x555 */
      CAN0->RAMn[ 0 * MSG_BUF_SIZE + 0] = 0x0C400000 | 8 << CAN_WMBn_CS_DLC_SHIFT; /* MB0 word 0: */
    }

  }

}



void CAN2_IRQHandler(void)

{

  while ((CAN2->RAMn[ RxBufferNo * MSG_BUF_SIZE + 0] & 0x01000000) >> 24)  {} // htu makesure code field is not busy
  //  RxCODE   = (CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] & 0x07000000) >> 24;  /* Read CODE field */
  RxID     = (CAN2->RAMn[ RxBufferNo * MSG_BUF_SIZE + 1] & CAN_WMBn_ID_ID_MASK)  >> 18 ; // shift 18 bit to get 11 standard ID
  // determin RTR
  RxLENGTH = (CAN2->RAMn[ RxBufferNo * MSG_BUF_SIZE + 0] & CAN_WMBn_CS_DLC_MASK) >> CAN_WMBn_CS_DLC_SHIFT;
  for (j = 0; j < 2; j++) { /* Read two words of data (8 bytes) */
    RxDATA[j] = CAN2->RAMn[ RxBufferNo * MSG_BUF_SIZE + 2 + j];
  }

  RxTIMESTAMP = (CAN2->RAMn[ RxBufferNo * MSG_BUF_SIZE + 0] & 0x000FFFF);
  dummy = CAN2->TIMER;             /* Read TIMER to unlock message buffers */
  CAN2->IFLAG1 = 0x00008000;       /* Clear CAN 2 MB 4 flag without clearing others*/


  //          CAN1->IFLAG1 = 0x00000001;       /* Clear CAN 0 MB 0 flag without clearing others*/
  CAN1->RAMn[ TxbufferNo * MSG_BUF_SIZE + 2] =  RxDATA[0]; // Insert Target Angle
  CAN1->RAMn[ TxbufferNo * MSG_BUF_SIZE + 3] =  RxDATA[1]; /* MB0 word 3: data word 1 */
  CAN1->RAMn[ TxbufferNo * MSG_BUF_SIZE + 1] = CAN2->RAMn[ RxBufferNo * MSG_BUF_SIZE + 1]; /* MB0 word 1: Tx msg with STD ID 0x555 */

  CAN1->RAMn[ TxbufferNo * MSG_BUF_SIZE + 0] = 0x0C400000 | RxLENGTH << CAN_WMBn_CS_DLC_SHIFT;
  TxbufferNo++;
  if (TxbufferNo >= BufLimit) {
    TxbufferNo = 0; //cycle TX buffer
  }
  //  }/* CODE=0xC: Activate msg buf to transmit */
  /* IDE=0: Standard ID */
  /* SRR=1 Tx frame (not req'd for std ID) */
  /* RTR = 0: data, not remote tx request frame*/
  /* DLC = 8 bytes */

  if ((RxID == 0x216) || (RxID == 0x217)||(RxID == 0x85)||(RxID == 0x82)) //relay EPAS info to CAN0 for monitoring
  {
    //PINS_DRV_TogglePins(LED_GPIO_PORT, PORT_LED1_MASK);
    //if(RxID==0x216){PINS_DRV_TogglePins(LED_GPIO_PORT, PORT_LED1_MASK);}
    if (RxID == 0x82) {
      SteeringControlStatus = RxDATA[0] >> 14 & 0x3;
      if (SteeringControlStatus == 0x3 || SteeringControlStatus == 0x0)
      {
        SteerByWire = 0; //reset Steer by wire due to error or override by driver.
      }
    } //DBC status1 indicate steering wheel control status 0x0 close, 0x1open,0x2 active,0x3Fault.
    CAN0->RAMn[ 0 * MSG_BUF_SIZE + 2] =  RxDATA[0]; // Insert Target Angle
    CAN0->RAMn[ 0 * MSG_BUF_SIZE + 3] =  RxDATA[1]; /* MB0 word 3: data word 1 */
    CAN0->RAMn[ 0 * MSG_BUF_SIZE + 1] = CAN2->RAMn[ RxBufferNo * MSG_BUF_SIZE + 1]; /* MB0 word 1: Tx msg with STD ID 0x555 */
    CAN0->RAMn[ 0 * MSG_BUF_SIZE + 0] = 0x0C400000 | RxLENGTH << CAN_WMBn_CS_DLC_SHIFT;
  }


}







void PORT_init (void) {
  PCC->PCCn[PCC_PORTC_INDEX ] |= PCC_PCCn_CGC_MASK; /* Enable clock for PORTC */
  PCC->PCCn[PCC_PORTA_INDEX ] |= PCC_PCCn_CGC_MASK; /* Enable clock for PORTA */ //  HTU
  PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clock for PORTE */
  PORTC->PCR[6] |= PORT_PCR_MUX(2);         /* Port C6: MUX = ALT2,UART1 TX */
  PORTC->PCR[7] |= PORT_PCR_MUX(2);         /* Port C7: MUX = ALT2,UART1 RX */

  PORTC->PCR[16] |= PORT_PCR_MUX(3);           /* Port C16: MUX = ALT3,CAN2 RX */ //  HTU
  PORTC->PCR[17] |= PORT_PCR_MUX(3);           /* Port C17: MUX = ALT3,CAN2 TX */ //  HTU
  PORTA->PCR[12] |= PORT_PCR_MUX(3);           /* Port C16: MUX = ALT3,CAN1 RX */ //  HTU
  PORTA->PCR[13] |= PORT_PCR_MUX(3);           /* Port C17: MUX = ALT3,CAN1 TX */ //  HTU


  PORTE->PCR[4] |= PORT_PCR_MUX(5); /* Port E4: MUX = ALT5, CAN0_RX */
  PORTE->PCR[5] |= PORT_PCR_MUX(5); /* Port E5: MUX = ALT5, CAN0_TX */
  PCC->PCCn[PCC_PORTD_INDEX ] |= PCC_PCCn_CGC_MASK; /* Enable clock for PORTD */
  PORTD->PCR[16] =  0x00000100;     /* Port D16: MUX = GPIO (to green LED) */
  PTD->PDDR |= 1 << 16;             /* Port D16: Data direction = output */
}



/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
   - startup asm routine
   - main()
*/
int main(void)
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
#ifdef PEX_RTOS_INIT
  PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
#endif
  /*** End of Processor Expert internal initialization.                    ***/

  /* Initialize and configure clocks
     -   Setup system clocks, dividers
     -   Configure LPUART clock, GPIO clock
     -   see clock manager component for more details
  */
  CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
                 g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
  CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

  /* Initialize pins
      -   Init LPUART and GPIO pins
      -   See PinSettings component for more info
  */
  PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

#if USE_LIN_XCVR
  /* Set LIN transceiver sleep pin direction */
  PINS_DRV_SetPinsDirection(LIN_XCVR_ENABLE_GPIO_PORT, LIN_XCVR_ENABLE_MASK);
  /* Wake up LIN transceiver */
  PINS_DRV_SetPins(LIN_XCVR_ENABLE_GPIO_PORT, LIN_XCVR_ENABLE_MASK);
#endif





  //uart

  PORT_init();           /* Configure ports */
  FLEXCAN0_init();         /* Init FlexCAN0 */
  FLEXCAN1_init();      //  HTU
  FLEXCAN2_init();      //  HTU
  LPUART1_init();        /* Initialize LPUART @ 9600*/
  LPUART1_transmit_string("initialized and code is running\n\r");     /* Transmit char string */
  //  LPUART1_transmit_string("Input character to echo...\n\r"); /* Transmit char string */

  //uart



  /* Enable button interrupt */
  INT_SYS_InstallHandler(BTN_PORT_IRQn, BTNPORT_IRQHandler, (isr_t *)NULL);
  INT_SYS_EnableIRQ(BTN_PORT_IRQn);

  //htu enable CAN0 interrupt
  INT_SYS_InstallHandler(CAN0_ORed_0_15_MB_IRQn, CAN0_IRQHandler, (isr_t *)NULL); //htu
  INT_SYS_EnableIRQ(CAN0_ORed_0_15_MB_IRQn); //htu
  //htu enable CAN1 interrupt
  INT_SYS_InstallHandler(CAN1_ORed_0_15_MB_IRQn, CAN1_IRQHandler, (isr_t *)NULL); //htu
  INT_SYS_EnableIRQ(CAN1_ORed_0_15_MB_IRQn); //htu
  //htu enable CAN2 interrupt
  INT_SYS_InstallHandler(CAN2_ORed_0_15_MB_IRQn, CAN2_IRQHandler, (isr_t *)NULL); //htu
  INT_SYS_EnableIRQ(CAN2_ORed_0_15_MB_IRQn); //htu

  /* Initialize LPTMR */
  //    LPTMR_DRV_Init(INST_LPTMR1, &lpTmr1_config0, false);
  //    INT_SYS_InstallHandler(LPTMR0_IRQn, LPTMR_ISR, (isr_t *)NULL);
  //    INT_SYS_EnableIRQ(LPTMR0_IRQn);
  //    LPTMR_DRV_StartCounter(INST_LPTMR1);
  PINS_DRV_SetPins(LED_GPIO_PORT, PORT_LED1_MASK);
  PINS_DRV_SetPins(LED_GPIO_PORT, PORT_LED2_MASK);
  PINS_DRV_SetPins(LED_GPIO_PORT, PORT_LED0_MASK);






  /* Start LIN master task */
  //  lin_master_task();

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the Freescale S32K series of microcontrollers.
**
** ###################################################################
*/
