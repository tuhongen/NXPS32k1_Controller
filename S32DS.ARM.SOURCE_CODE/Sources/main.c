/* ###################################################################
**     Filename    : main.c
**     Processor   : S32K1xx
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


/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "clockMan1.h"
#include "dmaController1.h"
#include "lpspiCom1.h"
#include "flexTimer_pwm1.h"
#include "flexTimer_pwm2.h"
#include "canCom0.h"
#include "canCom1.h"
#include "canCom2.h"
//#include "lpTmr1.h"
#include "lpit1.h"
#include "pin_mux.h"
#include "lpuart1.h"
#include "pwrMan1.h"

#if CPU_INIT_CONFIG
  #include "Init_Config.h"
#endif

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * Definitions
 ******************************************************************************/

/* This example is setup to work by default with EVB. To use it with other boards
   please comment the following line
*/

/* Definition of GPIO */
//input
#define ManualBrake 9U    //PTA 9  INPUT
#define ManualBrake_Port PTA    //PTA 9  INPUT
#define ManualAccel 15U     //PTE 15  INPUT
#define ManualAccel_Port PTE     //PTE 15  INPUT
//output
#define AutoBrakeStatus 0U   //PTD 0         OUTPUT
#define AutoBrakeStatus_Port PTD   //PTD 0         OUTPUT
//#define AutoAccelStatus 3U
#define AutoAccelStatus 16U  //PTE 16       OUTPUT
#define AutoAccelStatus_Port PTE  //PTE 16       OUTPUT
#define cs1 LPSPI_PCS1           //PTA 6        OUTPUT
#define cs1_Port PTA           //PTA 6        OUTPUT
#define cs2 LPSPI_PCS2       //PTA 16       OUTPUT
#define cs2_Port PTA         //PTA 16       OUTPUT
#define LED 13U        //PTC 13
#define LED_Port PTC         //PTC 13
//#define LED2 16        //PTD 16  LED2 on EVK

/* Definition of SPI */

#define NUMBER_OF_FRAMES 1U
#define BUFFER_SIZE 4U
#define TIMEOUT 5U


/* Definition of CAN */

//uint8_t TX_MAILBOX =0;
#define TX_MSG_ID   (1UL)

#define RX_MSG_ID   (2UL)

uint8_t TX_MAILBOX0  =0;
uint8_t TX_MAILBOX1  =0;
uint8_t TX_MAILBOX2  =0;
#define RX_MAILBOX0  (14U)
#define RX_MAILBOX1  (14U)
#define RX_MAILBOX2  (14U)

/* LPIT channel used */
#define LPIT_CHANNEL0        0UL
#define LPIT_CHANNEL1        1UL
#define LPIT_CHANNEL2        2UL
#define LPIT_CHANNEL3        3UL
#define LPIT_Channel0_IRQn   LPIT0_Ch0_IRQn
#define LPIT_Channel1_IRQn   LPIT0_Ch1_IRQn
#define LPIT_Channel2_IRQn   LPIT0_Ch2_IRQn
#define LPIT_Channel3_IRQn   LPIT0_Ch3_IRQn

uint32_t RxID  ;
uint32_t    RxLENGTH ;
uint32_t RxDATA[2];
uint32_t dummy;
uint32_t RxTIMESTAMP;
uint8_t TxbufferNo=0,BufLimit=15,RxBufferNo=15;//initial CAN rx buffer

uint32_t  CntrlMsgID = 0x160;
uint32_t  StateMsgID = 0x161;
uint32_t  n=0;
/* Define send buffer */
flexcan_msgbuff_t sendBuff;
/* Define receive buffer */
flexcan_msgbuff_t recvBuff;
/* Define send buffer of CAN1*/
flexcan_msgbuff_t sendCAN1Buff;
flexcan_msgbuff_t recvCAN1Buff;
/* Define send buffer of CAN2 */
flexcan_msgbuff_t sendCAN2Buff;
flexcan_msgbuff_t recvCAN2Buff;

bool CAN0Received=0;
bool CAN0TxComplete=0;

uint16_t  tem=0;
uint16_t  accelVol=0;
uint16_t  brakePWM=0;
uint16_t  volNew=0;
uint16_t  pwmNew=0;
uint8_t   autoAccel=0;
uint8_t   autoBrake=0;
uint8_t   manualCmd=0;
uint8_t   counterEN=0;
bool    CAN0_response=1;

/* Variables used for SPI */
uint16_t  voltageA, voltageB;
uint16_t  volAl  =   0x28B; // 0x147 initial Value 402mV,         0.8V*813.2037=651=0x28B
uint16_t  volBl  =   0x152;  //0xa8 initial Value 206mV,                 0.416V*813.2037=  338=0x152
uint16_t  volAu  =   0xc53;  //0x635 initial Value 1.954V= 813.2037count,  3.88V*813.2037=3155  =0xc53
uint16_t  volBu  =   0x65A;  //0x333 initial Value 1.006v,        2V*813.2037=1626=0x65A
bool        refreshSPI=0;

/* Variables used for PWM */
uint16_t  dutyCycleA, dutyCycleB;
uint16_t  CycleAl  =   28115U;
uint16_t  CycleBl  =   4620U;
uint16_t  CycleAu  =   19300U;
uint16_t  CycleBu  =   13435U;
ftm_state_t ftmStateStruct;

uint32_t pedalPressed=0;


/////////////////////////////////////////////////////////////////
uint32_t  AngleLimitP = 0x396C, AngleLimitN = 0x14B4;

int32_t  Error;
float speed, area, delta_t = 0.02, ControlAngle;
float decel_limit = 2500, accel_limit = 2500, speed_limit = 2500;
int32_t disR_low,disL_low,disR_low_p,disR_h,disL_low_p,disL_h;
int32_t disR_low_t,disL_low_t,deltaR,deltaL;
int32_t disR, disL;
uint32_t INT_Control_Angle; //CONVERT ControlAngle to uint32
int16_t CurrentSteeringAngle, TargetSteeringAngle = 0x2779;
int16_t preTSA = 0x2779 , deltaTSA = 0, rateTSA_acc = 10, rateTSA_dec = 10, rateTSA_spd = 10;
uint8_t j, SteeringControlStatus, EnableTransition = 0;
uint32_t counter = 0;
uint16_t timerOverflowInterruptCount = 0U;
uint8_t Node;
int16_t to16Buf;
long double Testing;
bool SteerByWire = 0;

uint8_t GearShift;

int32_t RxL, RxH;

uint8_t checks;
uint8_t RxL_n,RxH_n,delta_L,delta_H;
uint16_t RxLH;
int16_t RxE;
int16_t spd=0x0;

//gear shift
uint8_t GearShiftM;
uint8_t GearShiftRead;
uint8_t CounterRead,CounterReadH,CounterReadL;
uint8_t rorRead;
uint8_t GearCS;
uint8_t newCS;
bool GS_flag = 0;
uint8_t PFirst=0;
uint8_t GearCSOri;

//response
uint16_t R_angle=0;
uint8_t R_shift=0;
uint16_t R_speed=0;
uint16_t R_accel=0;
uint32_t R_brake_H=0,R_brake_L=0;


// UART Receive buffer size
#define UART_BUFFER_SIZE 256U

/* Buffer used to receive data from the console */
uint8_t uart_buffer[UART_BUFFER_SIZE];
uint8_t uart_bufferIdx;
bool uart_debug=1;
/* Welcome message displayed at the uart console */
#define welcomeMsg "MCU is starting and ready to use\r\n"
#define DebugMsgSize  30U
uint8_t DebugMsg1[DebugMsgSize] ="1:                          \r\n";
//uint32_t UARTDebugMSG_ADDR= *DebugMsg1;
uint8_t DebugMsg2[DebugMsgSize] ="2:                          \r\n";
uint8_t DebugMsg3[DebugMsgSize] ="3:                          \r\n";
uint8_t DebugMsg4[DebugMsgSize] ="4:                          \r\n";
uint8_t DebugMsg5[DebugMsgSize] ="5:                          \r\n";
uint8_t DebugMsg6[DebugMsgSize] ="6:                          \r\n";
bool OnesecondISR_Toggle =1;
bool UART_MSG_Clear;
bool UART_STATUS_OPEN;
uint32_t bytesRemaining;
uint8_t UARTCOUNT=0;
/////////////////////////////////////////////////////////////////

volatile bool dataReceived = false;
volatile bool manualFlag = true;
volatile bool automaFlag = false;
volatile bool brakeActiv = false;
volatile bool accelActiv = false;

typedef struct
{
  uint32_t tx[NUMBER_OF_FRAMES];
  uint32_t rx[NUMBER_OF_FRAMES];
} spi_buffer_t;

spi_buffer_t  master_buffer, slave_buffer;

flexcan_data_info_t dataInfo =
{
  .data_length = 8u,
  .msg_id_type = FLEXCAN_MSG_ID_STD,
  .enable_brs  = true,
  .fd_enable   = false,
  .fd_padding  = 0U
};

/******************************************************************************
 * Function prototypes
 ******************************************************************************/

/* Initialization */
void BoardInit(void);
void GPIOInit(void);
void FlexCANInit(void);
void LPSPInit(void);
void LPUARTInit(void);
void PWMInit(void);
void LPTMRInit(void);
void FLEXCAN0_init(void);
void FLEXCAN1_init(void);
void FLEXCAN2_init(void);

void CAN0_IRQHandler(void);
void CAN1_IRQHandler(void);
void CAN2_IRQHandler(void);

/* Function */
void SendCANData(uint8_t can_channel, uint8_t mailbox, uint32_t messageId, uint8_t * data);
void SendCANDataCycleMb(uint8_t can_channel, uint32_t messageId, uint8_t * data);
void UpdateSPI(uint16_t value, uint8_t pin_index);
void InitSPIBuffer(spi_buffer_t * spiBuffer, bool master);
void AutoControl(void);
void sendUART_debug(void);
/* Interrupt handler */

void processCAN0(void);
void processCAN1(void);
void processCAN2(void);
void CAN0receiveCallBack(uint8_t instance, flexcan_event_type_t eventType,flexcan_state_t *flexcanState);
void CAN1receiveCallBack(uint8_t instance, flexcan_event_type_t eventType,flexcan_state_t *flexcanState);
void CAN2receiveCallBack(uint8_t instance, flexcan_event_type_t eventType,flexcan_state_t *flexcanState);
void Brake_AutoManualISR(void);
void ACC_AutoManualISR(void);
void LPIT_ISR_50ms(void);
void LPIT_ISR_500ms(void);
void LPIT_ISR_1s(void);
void LPIT_ISR_2s(void);
void uartRXCallback(void *driverState, uart_event_t event, void *userData);

/******************************************************************************
 * Functions
 ******************************************************************************/

/*
 * @brief : Initialize clocks, pins and power modes
 */
void BoardInit(void)
{
    /* Initialize and configure clocks
     *  -   Setup system clocks, dividers
     *  -   Configure FlexCAN clock, GPIO
     *  -   see clock manager component for more details
     */
   CLOCK_DRV_Init(&clockMan1_InitConfig0);
//   POWER_SYS_Init(&pwrMan1_InitConfig0);
//  CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
//                        g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
//    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_FORCIBLE);

    /* Initialize pins
     *  -   Init LPSPI, FTM, FlexCAN, GPIO and LPTMR pins
     *  -   See PinSettings component for more info
     */
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
}

/*
 * @brief Function which configures the input and output
 */
void GPIOInit(void)
{
  /* Configure SPI chip select output
   *  -   Disable SPI chip select
   *  -   Set output high
   */
  PINS_DRV_WritePin(PTA,cs1,1);
  PINS_DRV_WritePin(PTD,cs2,1);

  /* Configure Manual-Auto mode
   *  -   Disable auto mode
   *  -   Set auto mode output low
   */
    PINS_DRV_ClearPins(PTD, 1 << AutoBrakeStatus);
//  PINS_DRV_ClearPins(PTA, 1 << AutoAccelStatus);
  PINS_DRV_ClearPins(PTE, 1 << AutoAccelStatus);

  /* Configure Manual mode interrupt
   *  -   Clear manual mode input
   *  -   Set manual mode input low
   */
//  PINS_DRV_ClearPins(PTA, 1 << ManualBrake);
//  PINS_DRV_ClearPins(PTE, 1 << ManualAccel);

    /* Setup manual mode input pins interrupt */
    PINS_DRV_SetPinIntSel(PORTA, ManualBrake, PORT_INT_RISING_EDGE);
    PINS_DRV_SetPinIntSel(PORTE, ManualAccel, PORT_INT_RISING_EDGE);

    /* Install manual mode checking ISR */
    INT_SYS_InstallHandler(PORTA_IRQn, &Brake_AutoManualISR, NULL);
    INT_SYS_InstallHandler(PORTE_IRQn, &ACC_AutoManualISR, NULL);
    /* Enable manual mode checking interrupt */
    INT_SYS_EnableIRQ(PORTA_IRQn);
    INT_SYS_EnableIRQ(PORTE_IRQn);
}

/*
 * @brief Initialize LPSPI driver
 */
void LPSPInit(void)
{
  /* Initialize and configure lpspi
  *  -   clock speed: 500 kHz
  *  -   bits/frame:  24
  *  -   pcs polarity:active low
  *  -   direction:   MSB first
  */
  LPSPI_DRV_MasterInit(LPSPICOM1, &lpspiCom1State, &lpspiCom1_MasterConfig0);

  /* Setup initial value */
  voltageB = volBl;  //initialize value to make sure updateSPI() uses initialized value
  voltageA = volAl; //initialize value
  UpdateSPI(volAl, cs1);
  UpdateSPI(volBl, cs2);
}

/*
 * @brief Initialize LPUART driver
 */
void LPUARTInit(void)
{

 /* Initialize LPUART instance */
  LPUART_DRV_Init(INST_LPUART1, &lpuart1_State, &lpuart1_InitConfig0);
  /* Install the callback for rx events */
  LPUART_DRV_InstallRxCallback(INST_LPUART1, uartRXCallback, NULL);
  // After calling UART_Init, you need to tell the driver to listen for a single byte

  LPUART_DRV_ReceiveData(INST_LPUART1, &uart_buffer[uart_bufferIdx], 1U);
  /* Send a welcome message */
  LPUART_DRV_SendData(INST_LPUART1, (uint8_t *)welcomeMsg, strlen(welcomeMsg));

}


/*
 * @brief Initialize PWM driver
 */
void PWMInit(void)
{
  /* Initialize and configure pwm1
  *  -   frequency: 526 Hz
  *  -   period value:11400
  *  -   duty cycle:  28115U(0x0~0x8000 -> 0%~100%)
  */
  /* Initialize FTM0 PWM1 channel 7 PTE9 */
  FTM_DRV_Init(INST_FLEXTIMER_PWM1, &flexTimer_pwm1_InitConfig, &ftmStateStruct);

  /* Initialize FTM0 PWM1 */
  FTM_DRV_InitPwm(INST_FLEXTIMER_PWM1, &flexTimer_pwm1_PwmConfig);

  /* Initialize and configure pwm2
  *  -   frequency: 476 Hz
  *  -   period value:12600
  *  -   duty cycle:  4620U(0x0~0x8000 -> 0%~100%)
  */
  /* Initialize FTM2 PWM2 channel 4 PTE10 */
  FTM_DRV_Init(INST_FLEXTIMER_PWM2, &flexTimer_pwm2_InitConfig, &ftmStateStruct);

  /* Initialize FTM2 PWM2 */
  FTM_DRV_InitPwm(INST_FLEXTIMER_PWM2, &flexTimer_pwm2_PwmConfig);
}


void FLEXCAN0_init(void) {
#define MSG_BUF_SIZE  4    /* Msg Buffer Size. (CAN 2.0AB: 2 hdr +  2 data= 4 words) */
  uint32_t   i=0;

  PCC->PCCn[PCC_FlexCAN0_INDEX] |= PCC_PCCn_CGC_MASK; /* CGC=1: enable clock to FlexCAN0 */
  CAN0->MCR |= CAN_MCR_MDIS_MASK;         /* MDIS=1: Disable module before selecting clock */

  CAN0->CTRL1 &= ~CAN_CTRL1_CLKSRC_MASK;  /* CLKSRC=0: Clock Source = oscillator (8 MHz) */
  CAN0->MCR &= ~CAN_MCR_MDIS_MASK;        /* MDIS=0; Enable module config. (Sets FRZ, HALT)*/
  while (!((CAN0->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT))  {}
                 /* Good practice: wait for FRZACK=1 on freeze mode entry/exit */
  CAN0->IMASK1=0x10; //htu enable MB4 interrupt and disable all others
  CAN0->CTRL1 = 0x00DB0006; /* Configure for 500 KHz bit time */
                            /* Time quanta freq = 16 time quanta x 500 KHz bit time= 8MHz */
                            /* PRESDIV+1 = Fclksrc/Ftq = 8 MHz/8 MHz = 1 */
                            /*    so PRESDIV = 0 */
                            /* PSEG2 = Phase_Seg2 - 1 = 4 - 1 = 3 */
                            /* PSEG1 = PSEG2 = 3 */
                            /* PROPSEG= Prop_Seg - 1 = 7 - 1 = 6 */
                            /* RJW: since Phase_Seg2 >=4, RJW+1=4 so RJW=3. */
                            /* SMP = 1: use 3 bits per CAN sample */
                            /* CLKSRC=0 (unchanged): Fcanclk= Fosc= 8 MHz */
  for(i=0; i<128; i++ ) {   /* CAN0: clear 32 msg bufs x 4 words/msg buf = 128 words*/
    CAN0->RAMn[i] = 0;      /* Clear msg buf word */
  }
  for(i=0; i<16; i++ ) {          /* In FRZ mode, init CAN0 16 msg buf filters */
 //   CAN0->RXIMR[i] = 0xFFFFFFFF;  /* Check all ID bits for incoming messages */
    CAN0->RXIMR[i] = 0x00000000; //htu
  }
 // CAN0->RXMGMASK = 0x1FFFFFFF;  /* Global acceptance mask: check all ID bits */
  CAN0->RXMGMASK = 0x00000000; //htu
  CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] = 0x04000000; /* Msg Buf 4, word 0: Enable for reception */
                                                /* EDL,BRS,ESI=0: CANFD not used */
                                                /* CODE=4: MB set to RX inactive */
                                                /* IDE=0: Standard ID */
                                                /* SRR, RTR, TIME STAMP = 0: not applicable */
#ifdef NODE_A                                   /* Node A receives msg with std ID 0x511 */
  CAN0->RAMn[ 4*MSG_BUF_SIZE + 1] = 0x14440000; /* Msg Buf 4, word 1: Standard ID = 0x111 */
#else                                           /* Node B to receive msg with std ID 0x555 */
  CAN0->RAMn[ 4*MSG_BUF_SIZE + 1] = 0x15540000; /* Msg Buf 4, word 1: Standard ID = 0x555 */
#endif
                                                /* PRIO = 0: CANFD not used */
  CAN0->IMASK1 = 0x00000010;//htu enable MB4 interrupt
  CAN0->MCR =0x0000001F|CAN_MCR_SRXDIS_MASK; //htu disable self receiption
  // CAN0->MCR = 0x0000001F;       /* Negate FlexCAN 1 halt state for 32 MBs */
  while ((CAN0->MCR && CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT)  {}
                 /* Good practice: wait for FRZACK to clear (not in freeze mode) */
  while ((CAN0->MCR && CAN_MCR_NOTRDY_MASK) >> CAN_MCR_NOTRDY_SHIFT)  {}
                 /* Good practice: wait for NOTRDY to clear (module ready)  */
}

void FLEXCAN1_init(void) {
#define MSG_BUF_SIZE  4    /* Msg Buffer Size. (CAN 2.0AB: 2 hdr +  2 data= 4 words) */
  uint32_t   i=0;

  PCC->PCCn[PCC_FlexCAN1_INDEX] |= PCC_PCCn_CGC_MASK; /* CGC=1: enable clock to FlexCAN0 */
  CAN1->MCR |= CAN_MCR_MDIS_MASK;         /* MDIS=1: Disable module before selecting clock */

  CAN1->CTRL1 &= ~CAN_CTRL1_CLKSRC_MASK;  /* CLKSRC=0: Clock Source = oscillator (8 MHz) */
  CAN1->MCR &= ~CAN_MCR_MDIS_MASK;        /* MDIS=0; Enable module config. (Sets FRZ, HALT)*/
  while (!((CAN1->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT))  {}
                 /* Good practice: wait for FRZACK=1 on freeze mode entry/exit */

 // CAN1->IMASK1=0x8000; //htu enable MB4 interrupt and disable all others
  CAN1->CTRL1 = 0x00DB0006|CAN_CTRL1_LBUF_MASK; /* Configure for 500 KHz bit time */
                            /* Time quanta freq = 16 time quanta x 500 KHz bit time= 8MHz */
                            /* PRESDIV+1 = Fclksrc/Ftq = 8 MHz/8 MHz = 1 */
                            /*    so PRESDIV = 0 */
                            /* PSEG2 = Phase_Seg2 - 1 = 4 - 1 = 3 */
                            /* PSEG1 = PSEG2 = 3 */
                            /* PROPSEG= Prop_Seg - 1 = 7 - 1 = 6 */
                            /* RJW: since Phase_Seg2 >=4, RJW+1=4 so RJW=3. */
                            /* SMP = 1: use 3 bits per CAN sample */
                            /* CLKSRC=0 (unchanged): Fcanclk= Fosc= 8 MHz */
  for(i=0; i<64; i++ ) {   /* CAN0: clear 32 msg bufs x 4 words/msg buf = 128 words*/
    CAN1->RAMn[i] = 0;      /* Clear msg buf word */
  }
  for(i=0; i<16; i++ ) {          /* In FRZ mode, init CAN0 16 msg buf filters */
  //  CAN1->RXIMR[i] = 0xFFFFFFFF;  /* Check all ID bits for incoming messages */
    CAN1->RXIMR[i] = 0x00000000;
  }
//  CAN1->RXMGMASK = 0x1FFFFFFF;  /* Global acceptance mask: check all ID bits */
  CAN1->RXMGMASK = 0x00000000;
  CAN1->RX15MASK = 0x00000000;
  CAN1->RAMn[ 15*MSG_BUF_SIZE + 0] = 0x04000000; /* Msg Buf 4, word 0: Enable for reception */
                                                /* EDL,BRS,ESI=0: CANFD not used */
                                                /* CODE=4: MB set to RX inactive */
                                                /* IDE=0: Standard ID */
                                                /* SRR, RTR, TIME STAMP = 0: not applicable */
#ifdef NODE_A                                   /* Node A receives msg with std ID 0x511 */
  CAN1->RAMn[ 15*MSG_BUF_SIZE + 1] = 0x14440000; /* Msg Buf 4, word 1: Standard ID = 0x111 */
#else                                           /* Node B to receive msg with std ID 0x555 */
  CAN1->RAMn[ 15*MSG_BUF_SIZE + 1] = 0x15540000; /* Msg Buf 4, word 1: Standard ID = 0x555 */
#endif
                                                /* PRIO = 0: CANFD not used */
  CAN1->IMASK1 = 0x00008000;//htu enable MB4 interrupt
  CAN1->MCR =0x0000000F|CAN_MCR_SRXDIS_MASK|CAN_MCR_LPRIOEN_MASK ; //htu disable self receiption
  // CAN1->MCR = 0x0000000F;       /* Negate FlexCAN 1 halt state for 32 MBs */
  while ((CAN1->MCR && CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT)  {}
                 /* Good practice: wait for FRZACK to clear (not in freeze mode) */
  while ((CAN1->MCR && CAN_MCR_NOTRDY_MASK) >> CAN_MCR_NOTRDY_SHIFT)  {}
                 /* Good practice: wait for NOTRDY to clear (module ready)  */
}

void FLEXCAN2_init(void)
{
#define MSG_BUF_SIZE  4    /* Msg Buffer Size. (CAN 2.0AB: 2 hdr +  2 data= 4 words) */
  uint32_t   i=0;

  PCC->PCCn[PCC_FlexCAN2_INDEX] |= PCC_PCCn_CGC_MASK; /* CGC=1: enable clock to FlexCAN0 */
  CAN2->MCR |= CAN_MCR_MDIS_MASK;         /* MDIS=1: Disable module before selecting clock */

  CAN2->CTRL1 &= ~CAN_CTRL1_CLKSRC_MASK;  /* CLKSRC=0: Clock Source = oscillator (8 MHz) */
  CAN2->MCR &= ~CAN_MCR_MDIS_MASK;        /* MDIS=0; Enable module config. (Sets FRZ, HALT)*/
  while (!((CAN2->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT))  {}
                 /* Good practice: wait for FRZACK=1 on freeze mode entry/exit */

 // CAN2->IMASK1=0x8000; //htu enable MB15 interrupt and disable all others
  CAN2->CTRL1 = 0x00DB0006|CAN_CTRL1_LBUF_MASK; /* Configure for 500 KHz bit time */
                            /* Time quanta freq = 16 time quanta x 500 KHz bit time= 8MHz */
                            /* PRESDIV+1 = Fclksrc/Ftq = 8 MHz/8 MHz = 1 */
                            /*    so PRESDIV = 0 */
                            /* PSEG2 = Phase_Seg2 - 1 = 4 - 1 = 3 */
                            /* PSEG1 = PSEG2 = 3 */
                            /* PROPSEG= Prop_Seg - 1 = 7 - 1 = 6 */
                            /* RJW: since Phase_Seg2 >=4, RJW+1=4 so RJW=3. */
                            /* SMP = 1: use 3 bits per CAN sample */
                            /* CLKSRC=0 (unchanged): Fcanclk= Fosc= 8 MHz */
  for(i=0; i<64; i++ ) {   /* CAN0: clear 32 msg bufs x 4 words/msg buf = 128 words*/
    CAN2->RAMn[i] = 0;      /* Clear msg buf word */
  }
  for(i=0; i<16; i++ ) {          /* In FRZ mode, init CAN0 16 msg buf filters */
  //  CAN1->RXIMR[i] = 0xFFFFFFFF;  /* Check all ID bits for incoming messages */
    CAN2->RXIMR[i] = 0x00000000;
  }
//  CAN1->RXMGMASK = 0x1FFFFFFF;  /* Global acceptance mask: check all ID bits */
  CAN2->RXMGMASK = 0x00000000;
  CAN2->RX15MASK= 0x00000000;
  CAN2->RAMn[ 15*MSG_BUF_SIZE + 0] = 0x04000000; /* Msg Buf 15, word 0: Enable for reception */
                                                /* EDL,BRS,ESI=0: CANFD not used */
                                                /* CODE=4: MB set to RX inactive */
                                                /* IDE=0: Standard ID */
                                                /* SRR, RTR, TIME STAMP = 0: not applicable */
#ifdef NODE_A                                   /* Node A receives msg with std ID 0x511 */
  CAN1->RAMn[ 15*MSG_BUF_SIZE + 1] = 0x14440000; /* Msg Buf 4, word 1: Standard ID = 0x111 */
#else                                           /* Node B to receive msg with std ID 0x555 */
  CAN1->RAMn[ 4*MSG_BUF_SIZE + 1] = 0x15540000; /* Msg Buf 4, word 1: Standard ID = 0x555 */
#endif
                                                /* PRIO = 0: CANFD not used */
  CAN2->IMASK1 = 0x00008000;//htu enable MB4 interrupt
  CAN2->MCR =0x0000000F|CAN_MCR_SRXDIS_MASK|CAN_MCR_LPRIOEN_MASK ; //htu disable self receiption
  // CAN1->MCR = 0x0000000F;       /* Negate FlexCAN 1 halt state for 32 MBs */
  while ((CAN2->MCR && CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT)  {}
                 /* Good practice: wait for FRZACK to clear (not in freeze mode) */
  while ((CAN2->MCR && CAN_MCR_NOTRDY_MASK) >> CAN_MCR_NOTRDY_SHIFT)  {}
                 /* Good practice: wait for NOTRDY to clear (module ready)  */
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
}

/*
 * @brief Initialize FlexCAN driver and configure the bit rate
 */
void FlexCANInit(void)
{
//  FLEXCAN_DRV_Init(INST_CANCOM0, &canCom0_State, &canCom0_InitConfig0);
//    FLEXCAN_DRV_Init(INST_CANCOM1, &canCom1_State, &canCom1_InitConfig0);
//    FLEXCAN_DRV_Init(INST_CANCOM2, &canCom2_State, &canCom2_InitConfig0);

//    FLEXCAN0_init();
//  FLEXCAN1_init();
//  FLEXCAN2_init();
//  INT_SYS_InstallHandler(CAN0_ORed_0_15_MB_IRQn, CAN0_IRQHandler, (isr_t *)NULL); //htu
//  INT_SYS_EnableIRQ(CAN0_ORed_0_15_MB_IRQn); //htu
//  INT_SYS_InstallHandler(CAN1_ORed_0_15_MB_IRQn, CAN1_IRQHandler, (isr_t *)NULL); //htu
//  INT_SYS_EnableIRQ(CAN1_ORed_0_15_MB_IRQn); //htu
//  //htu enable CAN2 interrupt
//  INT_SYS_InstallHandler(CAN2_ORed_0_15_MB_IRQn, CAN2_IRQHandler, (isr_t *)NULL); //htu
//  INT_SYS_EnableIRQ(CAN2_ORed_0_15_MB_IRQn); //htu
    /* Initialize FlexCAN driver
     *  - 8 byte payload size
     *  - FD disabled
     *  - Bus clock as peripheral engine clock
     */
	FLEXCAN_DRV_Init(INST_CANCOM0, &canCom0_State, &canCom0_InitConfig0);
    FLEXCAN_DRV_Init(INST_CANCOM1, &canCom1_State, &canCom1_InitConfig0);
    FLEXCAN_DRV_Init(INST_CANCOM2, &canCom2_State, &canCom2_InitConfig0);

    /* Set information about the data to be sent
   *  - 8 byte in length
   *  - Standard message ID
   *  - Bit rate switch enabled to use a different bit rate for the data segment
   *  - Flexible data rate disabled
   *  - Use zeros for FD padding
   */
  flexcan_data_info_t dataInfo =
  {
    .data_length = 8u,
    .msg_id_type = FLEXCAN_MSG_ID_STD,
    .enable_brs  = true,
    .fd_enable   = false,
    .fd_padding  = 0U
  };

//  sendBuff.data[0]=0x11;
  /* Execute send non-blocking */
//  FLEXCAN_DRV_Send(INST_CANCOM0, TX_MAILBOX0, &dataInfo, 0x123, sendBuff.data);
  //FLEXCAN_DRV_Send(INST_CANCOM1, TX_MAILBOX1, &dataInfo, 0x123, sendBuff.data);
  //FLEXCAN_DRV_Send(INST_CANCOM2, TX_MAILBOX2, &dataInfo, 0x123, sendBuff.data);
//  sendBuff.data[0]=0x0;

//  /* Configure RX message buffer with index RX_MSG_ID and RX_MAILBOX */
//    CAN0->MCR &= ~CAN_MCR_MDIS_MASK;        /* MDIS=0; Enable module config. (Sets FRZ, HALT)*/
//    while (!((CAN0->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT))  {}
//    CAN0->RXMGMASK = 0x00000000; //htu
//     CAN0->RXIMR[0] = 0x00000000; //htu
//  /* Global acceptance mask: check all ID bits */
//    CAN0->RXMGMASK = 0x00000000; //htu
//    while ((CAN0->MCR && CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT)  {}
//                    /* Good practice: wait for FRZACK to clear (not in freeze mode) */
//     while ((CAN0->MCR && CAN_MCR_NOTRDY_MASK) >> CAN_MCR_NOTRDY_SHIFT)  {}
//                    /* Good practice: wait for NOTRDY to clear (module ready)  */

    FLEXCAN_DRV_InstallEventCallback(INST_CANCOM0, CAN0receiveCallBack, 0);
    FLEXCAN_DRV_InstallEventCallback(INST_CANCOM1, CAN1receiveCallBack, 0);
    FLEXCAN_DRV_InstallEventCallback(INST_CANCOM2, CAN2receiveCallBack, 0);

    FLEXCAN_DRV_ConfigRxMb(INST_CANCOM0, RX_MAILBOX0, &dataInfo, 0U );
    FLEXCAN_DRV_SetRxMb14Mask(INST_CANCOM0, FLEXCAN_MSG_ID_STD,0U);
//    FLEXCAN_DRV_SetRxIndividualMask(INST_CANCOM0, FLEXCAN_MSG_ID_STD,RX_MAILBOX0,0U);
   // FLEXCAN_DRV_SetRxMbGlobalMask(INST_CANCOM0, FLEXCAN_MSG_ID_STD,0U);
  //  FLEXCAN_DRV_SetRxIndividualMask(INST_CANCOM0, FLEXCAN_MSG_ID_STD,RX_MAILBOX0,0U);


    FLEXCAN_DRV_ConfigRxMb(INST_CANCOM1, RX_MAILBOX1, &dataInfo, 0U);
    FLEXCAN_DRV_SetRxMb14Mask(INST_CANCOM1, FLEXCAN_MSG_ID_STD,0U);
  //  FLEXCAN_DRV_SetRxIndividualMask(INST_CANCOM1, FLEXCAN_MSG_ID_STD,RX_MAILBOX1,0U);
  //  FLEXCAN_DRV_SetRxMbGlobalMask(INST_CANCOM1, FLEXCAN_MSG_ID_STD,0U);

    FLEXCAN_DRV_ConfigRxMb(INST_CANCOM2, RX_MAILBOX2, &dataInfo, 0U);
    FLEXCAN_DRV_SetRxMb14Mask(INST_CANCOM2, FLEXCAN_MSG_ID_STD,0U);
  //  FLEXCAN_DRV_SetRxIndividualMask(INST_CANCOM2, FLEXCAN_MSG_ID_STD,RX_MAILBOX2,0U);
  //  FLEXCAN_DRV_SetRxMbGlobalMask(INST_CANCOM2, FLEXCAN_MSG_ID_STD,0U);

    /* Start receiving data in RX_MAILBOX. */
    FLEXCAN_DRV_Receive(INST_CANCOM0, RX_MAILBOX0, &recvBuff);
    FLEXCAN_DRV_Receive(INST_CANCOM1, RX_MAILBOX1, &recvCAN1Buff);
    FLEXCAN_DRV_Receive(INST_CANCOM2, RX_MAILBOX2, &recvCAN2Buff);



}

void LPTMRInit(void)
{
    /* Initialize LPIT instance 0
     *  -   Reset and enable peripheral
     */
    LPIT_DRV_Init(INST_LPIT1, &lpit1_InitConfig);

    /* Initialize LPIT channel 0 and configure it as a periodic counter
     * which is used to generate an interrupt.
     */
    LPIT_DRV_InitChannel(INST_LPIT1, LPIT_CHANNEL0, &lpit1_ChnConfig0);
    LPIT_DRV_InitChannel(INST_LPIT1, LPIT_CHANNEL1, &lpit1_ChnConfig1);
    LPIT_DRV_InitChannel(INST_LPIT1, LPIT_CHANNEL2, &lpit1_ChnConfig2);
    LPIT_DRV_InitChannel(INST_LPIT1, LPIT_CHANNEL3, &lpit1_ChnConfig3);
    /* Install LPIT_ISR as LPIT interrupt handler */
    INT_SYS_InstallHandler(LPIT_Channel0_IRQn, &LPIT_ISR_50ms, (isr_t *)0);
    INT_SYS_InstallHandler(LPIT_Channel1_IRQn, &LPIT_ISR_500ms, (isr_t *)0);
    INT_SYS_InstallHandler(LPIT_Channel2_IRQn, &LPIT_ISR_1s, (isr_t *)0);
    INT_SYS_InstallHandler(LPIT_Channel3_IRQn, &LPIT_ISR_2s, (isr_t *)0);
    /* Start LPIT0 channel 0 counter */
    LPIT_DRV_StartTimerChannels(INST_LPIT1, (1 << LPIT_CHANNEL0));
    LPIT_DRV_StartTimerChannels(INST_LPIT1, (1 << LPIT_CHANNEL1));
    LPIT_DRV_StartTimerChannels(INST_LPIT1, (1 << LPIT_CHANNEL2));
    LPIT_DRV_StartTimerChannels(INST_LPIT1, (1 << LPIT_CHANNEL3));
    /* Peripherals Initialization is complete, now the program will wait for
     * LPIT interrupt.
     */

}


/*
 * @brief: Send data via CAN and cycle through mailbox 0-13 with the specified message id
  * @param messageId : Message ID
 * @param data      : Pointer to the TX data
 * @return          : None
 */
void SendCANDataCycleMb(uint8_t can_channel, uint32_t messageId, uint8_t * data)
{
  switch(can_channel)
    {
    case INST_CANCOM0:
      {
        /* Get Mailbox LOOP TRANSMISSION STATUS */
          if (FLEXCAN_DRV_GetTransferStatus(can_channel,TX_MAILBOX0) != STATUS_SUCCESS) {
            /* Ends a non-blocking FlexCAN transfer */
            FLEXCAN_DRV_AbortTransfer(can_channel, TX_MAILBOX0);
          }

            /* Configure TX message buffer with index TX_MSG_ID and TX_MAILBOX */
            FLEXCAN_DRV_ConfigTxMb(can_channel, TX_MAILBOX0, &dataInfo, messageId);

            /* Execute send non-blocking */
            FLEXCAN_DRV_Send(can_channel, TX_MAILBOX0, &dataInfo, messageId, data);

            if (TX_MAILBOX0<13U)
              {
              TX_MAILBOX0++;
              }
            else
              {
              TX_MAILBOX0=0;
              }
      }
      break;
    case INST_CANCOM1:
      {
          /* Get Mailbox LOOP TRANSMISSION STATUS */
             if (FLEXCAN_DRV_GetTransferStatus(can_channel,TX_MAILBOX1) != STATUS_SUCCESS) {
               /* Ends a non-blocking FlexCAN transfer */
               FLEXCAN_DRV_AbortTransfer(can_channel, TX_MAILBOX1);
             }

               /* Configure TX message buffer with index TX_MSG_ID and TX_MAILBOX */
               FLEXCAN_DRV_ConfigTxMb(can_channel, TX_MAILBOX1, &dataInfo, messageId);

               /* Execute send non-blocking */
               FLEXCAN_DRV_Send(can_channel, TX_MAILBOX1, &dataInfo, messageId, data);

               if (TX_MAILBOX1<13U)
                 {
                 TX_MAILBOX1++;
                 }
               else
                 {
                 TX_MAILBOX1=0;
                 }
      }
      break;
    case INST_CANCOM2:
      {
          /* Get Mailbox LOOP TRANSMISSION STATUS */
             if (FLEXCAN_DRV_GetTransferStatus(can_channel,TX_MAILBOX2) != STATUS_SUCCESS) {
               /* Ends a non-blocking FlexCAN transfer */
               FLEXCAN_DRV_AbortTransfer(can_channel, TX_MAILBOX2);
             }

               /* Configure TX message buffer with index TX_MSG_ID and TX_MAILBOX */
               FLEXCAN_DRV_ConfigTxMb(can_channel, TX_MAILBOX2, &dataInfo, messageId);

               /* Execute send non-blocking */
               FLEXCAN_DRV_Send(can_channel, TX_MAILBOX2, &dataInfo, messageId, data);

               if (TX_MAILBOX2<13U)
                 {
                 TX_MAILBOX2++;
                 }
               else
                 {
                 TX_MAILBOX2=0;
                 }
      }
      break;

    default:
      break;
    }


}



/*
 * @brief: Send data via CAN to the specified mailbox with the specified message id
 * @param mailbox   : Destination mailbox number
 * @param messageId : Message ID
 * @param data      : Pointer to the TX data
 * @param len       : Length of the TX data
 * @return          : None
 */
void SendCANData(uint8_t can_channel, uint8_t mailbox, uint32_t messageId, uint8_t * data)
{
  /* Get error status */
  if (FLEXCAN_DRV_GetErrorStatus(can_channel) != 0) {
    /* Ends a non-blocking FlexCAN transfer early */
    FLEXCAN_DRV_AbortTransfer(can_channel, mailbox);
  }

    /* Configure TX message buffer with index TX_MSG_ID and TX_MAILBOX */
    FLEXCAN_DRV_ConfigTxMb(can_channel, mailbox, &dataInfo, messageId);

    /* Execute send non-blocking */
    FLEXCAN_DRV_Send(can_channel, mailbox, &dataInfo, messageId, data);
}

/*
 * @brief: Update SPI data
 * @param value   : data
 * @param pin_index : chip select index of GPIO
 * @return          : None
 */
void UpdateSPI(uint16_t value, uint8_t pin_index)   //THIS FUNCTION can not called inside ISR.
{
  tem=value;
  InitSPIBuffer(&master_buffer, true);
  /* select chip */
  LPSPI_DRV_SetPcs(LPSPICOM1,pin_index,LPSPI_ACTIVE_LOW);

  LPSPI_DRV_MasterTransfer(LPSPICOM1, master_buffer.tx, NULL, BUFFER_SIZE);
  //PINS_DRV_TogglePins(PTE,1 << 4);
  while(LPSPI_DRV_MasterGetTransferStatus(LPSPICOM1, master_buffer.rx)>0)
  {
    // get out while loop once spi transfer finished
  }
  //PINS_DRV_TogglePins(PTE,1 << 4);
  //PINS_DRV_WritePin(PTA,pin_index,0);
//  LPSPI_DRV_MasterTransferBlocking(LPSPICOM1, &master_buffer.tx, NULL, BUFFER_SIZE, 1);
  //LPSPI_DRV_MasterTransfer(LPSPICOM1, master_buffer.tx, NULL, BUFFER_SIZE);
//  while(LPSPI_DRV_MasterGetTransferStatus(LPSPICOM1, master_buffer.rx)>0);
//  PINS_DRV_WritePin(PTA,pin_index,1);
//  LPSPI_DRV_MasterAbortTransfer(LPSPICOM1);
}

void AutoControl(void)
{
//  PINS_DRV_WritePin(PTD,15,1);

  /* Auto mode,and no manual overide*/
  if (automaFlag && !manualFlag){

    /* Enable auto mode by setting output high */
    if (accelActiv){
//      PINS_DRV_SetPins(PTA, 1 << AutoAccelStatus);
      PINS_DRV_SetPins(PTE, 1 << AutoAccelStatus);
//      PINS_DRV_ClearPins(PTA, 1 << AutoAccelStatus);
    } else {
//      PINS_DRV_ClearPins(PTA, 1 << AutoAccelStatus);
      PINS_DRV_ClearPins(PTE, 1 << AutoAccelStatus);
//      PINS_DRV_SetPins(PTA, 1 << AutoAccelStatus);
      accelVol = 0;
    }
    if (brakeActiv){
      PINS_DRV_SetPins(PTD, 1 << AutoBrakeStatus);
//      PINS_DRV_ClearPins(PTA, 1 << AutoBrakeStatus);
    } else {
      PINS_DRV_ClearPins(PTD, 1 << AutoBrakeStatus);
//      PINS_DRV_SetPins(PTA, 1 << AutoBrakeStatus);
      brakePWM = 0;
    }

    /* Update auto control command */
    if (brakePWM>0){
      accelVol=0x0;
      voltageA = volAl + (volAu-volAl)*accelVol/0xff;

      voltageB = volBl + (volBu-volBl)*accelVol/0xff;
      refreshSPI=1; // goes to main loop to perform updateSPI.
      /* select chip one */
    //  UpdateSPI(voltageA, cs1);
      /* select chip two */
    //  UpdateSPI(voltageB, cs2);

      dutyCycleA=CycleAl + (CycleAu-CycleAl)*brakePWM/0xff;
      FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM1, 7U, FTM_PWM_UPDATE_IN_DUTY_CYCLE, dutyCycleA, 0U, true);

      dutyCycleB=CycleBl + (CycleBu-CycleBl)*brakePWM/0xff;
      FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM2, 4U, FTM_PWM_UPDATE_IN_DUTY_CYCLE, dutyCycleB, 0U, true);
    } else {
      brakePWM=0;
      dutyCycleA=CycleAl + (CycleAu-CycleAl)*brakePWM/0xff;
      FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM1, 7U, FTM_PWM_UPDATE_IN_DUTY_CYCLE, dutyCycleA, 0U, true);

      dutyCycleB=CycleBl + (CycleBu-CycleBl)*brakePWM/0xff;
      FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM2, 4U, FTM_PWM_UPDATE_IN_DUTY_CYCLE, dutyCycleB, 0U, true);

      voltageA = volAl + (volAu-volAl)*accelVol/0xff;
      /* select chip one */
    //  UpdateSPI(voltageA, cs1);

      voltageB = volBl + (volBu-volBl)*accelVol/0xff;
      /* select chip two */
    //  UpdateSPI(voltageB, cs2);
      refreshSPI=1; // goes to main loop to perform updateSPI.
    }
  } else {
    accelVol=0;
    voltageA = volAl + (volAu-volAl)*accelVol/0xff;
    /* select chip one */
  //  UpdateSPI(voltageA, cs1);

    voltageB = volBl + (volBu-volBl)*accelVol/0xff;
    /* select chip two */
  //  UpdateSPI(voltageB, cs2);
    refreshSPI=1; // goes to main loop to perform updateSPI.

    brakePWM=0;
    dutyCycleA=CycleAl + (CycleAu-CycleAl)*brakePWM/0xff;
    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM1, 7U, FTM_PWM_UPDATE_IN_DUTY_CYCLE, dutyCycleA, 0U, true);

    dutyCycleB=CycleBl + (CycleBu-CycleBl)*brakePWM/0xff;
    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER_PWM2, 4U, FTM_PWM_UPDATE_IN_DUTY_CYCLE, dutyCycleB, 0U, true);
  }

//  PINS_DRV_ClearPins(PTD, 1 << 15);
}

void InitSPIBuffer(spi_buffer_t * spiBuffer, bool master)
{

  spiBuffer->tx[0] = tem<<4;

  spiBuffer->rx[0] = 0x0;
}




/* UART rx callback for continuous reception, byte by byte */
void uartRXCallback(void *driverState, uart_event_t event, void *userData)
{
    /* Unused parameters */
    (void)driverState;
    (void)userData;

    /* The reception stops when newline is received or the buffer is full */
       if ((uart_buffer[uart_bufferIdx] == '\n') || (uart_bufferIdx >= (UART_BUFFER_SIZE - 2U)))
         {// time to decode message and reset uartbufferIdx
           // if((uart_buffer[0] == &("O"))&&(uart_buffer[1]==&("N")))
         if((uart_buffer[0] == 79U)&&(uart_buffer[1] == 78U)  )       // 79 O, 78N, ASCII CODE
         {
             uart_debug=1;
               LPUART_DRV_SendData(INST_LPUART1, (uint8_t *)"Debug Turned ON\n", sizeof("Debug Turned ON\n"));//timeout asumed to be 1ms
            }
           // if((uart_buffer[0]==&("O"))&&(uart_buffer[1]==&("F"))&&(uart_buffer[2]==&("F")))
         if((uart_buffer[0] == 79U)&&(uart_buffer[1] == 70U)&&(uart_buffer[2] == 70U)  )  //70 F,79 O, ASCII CODE
         {
              uart_debug=0;
              LPUART_DRV_SendData(INST_LPUART1, (uint8_t *)"Debug Turned OFF\n", sizeof("Debug Turned OFF\n"));//timeout asumed to be 1ms
            }

         uart_bufferIdx=0;
          LPUART_DRV_SetRxBuffer(INST_LPUART1, &uart_buffer[uart_bufferIdx], 1U);// call this will enable uartRXCallback upon next reception.
         }
       else
       {    /* Update the buffer index and the rx buffer */
         uart_bufferIdx++; // next call back will save data to next uart_bufferIdx
         LPUART_DRV_SetRxBuffer(INST_LPUART1, &uart_buffer[uart_bufferIdx], 1U);// call this will enable uartRXCallback upon next reception.
       }

    /* Check the event type */
    if (event == UART_EVENT_RX_FULL)
    {

    }
}





void CAN0receiveCallBack(uint8_t instance, flexcan_event_type_t eventType, flexcan_state_t *flexcanState)
{
  (void)flexcanState;
  (void)instance;
  switch(eventType)
  {
  case FLEXCAN_EVENT_RX_COMPLETE:
    {

      processCAN0();
      FLEXCAN_DRV_Receive(INST_CANCOM0, RX_MAILBOX0, &recvBuff); // turn on receive to trigger interupt upon next CAN receive event.

      //CAN0Received = 1;
      //FLEXCAN_DRV_Receive(INST_CANCOM0, RX_MAILBOX0, &recvBuff);
    }
    break;
  case FLEXCAN_EVENT_RXFIFO_COMPLETE:
    break;
  case FLEXCAN_EVENT_DMA_COMPLETE:
    {
    //  rxFIFOdone = 1;
    }
    break;
  case FLEXCAN_EVENT_TX_COMPLETE:
    {
    //  CAN0TxComplete=1;
    }
    break;
  default:
    break;
  }


}


void CAN1receiveCallBack(uint8_t instance, flexcan_event_type_t eventType, flexcan_state_t *flexcanState)
{
  (void)flexcanState;
  (void)instance;
    switch(eventType)
    {
    case FLEXCAN_EVENT_RX_COMPLETE:
      {

        processCAN1();
        FLEXCAN_DRV_Receive(INST_CANCOM1, RX_MAILBOX1, &recvCAN1Buff);  // turn on receive to trigger interupt upon next CAN receive event.

        //CAN0Received = 1;
        //FLEXCAN_DRV_Receive(INST_CANCOM0, RX_MAILBOX0, &recvBuff);
      }
      break;
    case FLEXCAN_EVENT_RXFIFO_COMPLETE:
      break;
    case FLEXCAN_EVENT_DMA_COMPLETE:
      {
      //  rxFIFOdone = 1;
      }
      break;
    case FLEXCAN_EVENT_TX_COMPLETE:
      {
      //  CAN0TxComplete=1;
      }
      break;
    default:
      break;
    }



}

void CAN2receiveCallBack(uint8_t instance, flexcan_event_type_t eventType, flexcan_state_t *flexcanState)
{


  (void)flexcanState;
    (void)instance;
      switch(eventType)
      {
      case FLEXCAN_EVENT_RX_COMPLETE:
        {

          processCAN2();
          FLEXCAN_DRV_Receive(INST_CANCOM2, RX_MAILBOX2, &recvCAN2Buff);  // turn on receive to trigger interupt upon next CAN receive event.

          //CAN0Received = 1;
          //FLEXCAN_DRV_Receive(INST_CANCOM0, RX_MAILBOX0, &recvBuff);
        }
        break;
      case FLEXCAN_EVENT_RXFIFO_COMPLETE:
        break;
      case FLEXCAN_EVENT_DMA_COMPLETE:
        {
        //  rxFIFOdone = 1;
        }
        break;
      case FLEXCAN_EVENT_TX_COMPLETE:
        {
        //  CAN0TxComplete=1;
        }
        break;
      default:
        break;
      }
}



void processCAN0(void)
{
//	if (recvBuff.msgId == StateMsgID)
//	{
//	// check to see self receiption
//	}

    /* Update control message 0x160 */
    if (recvBuff.msgId == CntrlMsgID){
      if (autoAccel != recvBuff.data[2]){
        autoAccel = recvBuff.data[2];
        dataReceived=true;

        if (autoAccel==0x1){
          volNew = 0;
        } else {
          pwmNew = 0;
        }
      }
      if (autoBrake != recvBuff.data[6]){
        autoBrake = recvBuff.data[6];
        dataReceived=true;

        if (autoBrake==0x1){
          pwmNew = 0;
        } else {
          volNew = 0;
        }
      }

      if (manualCmd!=recvBuff.data[0]){
        manualCmd=recvBuff.data[0];
        manualFlag = false;

        accelVol = recvBuff.data[3];
        brakePWM = recvBuff.data[7];
        dataReceived=true;
      }

      if (!manualFlag){
        if (autoAccel==0x1){
          accelActiv = true;

          if (volNew!=recvBuff.data[3]) {
            accelVol = recvBuff.data[3];
            dataReceived=true;
            volNew = recvBuff.data[3];
          }
        } else {
          accelActiv = false;
        }

        if (autoBrake==0x1){
          brakeActiv = true;

          if (pwmNew!=recvBuff.data[7]) {
            brakePWM = recvBuff.data[7];
            dataReceived=true;
            pwmNew = recvBuff.data[7];
          }
        } else {
          brakeActiv = false;
        }

        if (accelActiv || brakeActiv){
          automaFlag = true;
        }
      }

      if (counterEN!=recvBuff.data[4]){
        counterEN = recvBuff.data[4];

        if (counterEN == 0x10){
          /* Start LPTMR counter */
  //        LPTMR_DRV_StartCounter(INST_LPTMR1);
          CAN0_response=1;
        } else {
          CAN0_response=0;
  //        LPTMR_DRV_StopCounter(INST_LPTMR1);
        }
      }
    }

    /* Update steering control message 0x112 */
      if (recvBuff.msgId == 0x112)
      {
    	//  if(uart_debug){LPUART_DRV_SendData(INST_LPUART1, (uint8_t *)"112\n", sizeof("112\n"));}
    	  if(uart_debug){ strcpy(DebugMsg1, "1: 112 cmd rcvd");}
    	//  strcpy(DebugMsg1, "test");
          if ((SteeringControlStatus == 0x01) || (SteeringControlStatus == 0x02)) //steering module open and under control
          {
              //to16Buf = RxDATA[0] & 0x0000FFFF;
              to16Buf = recvBuff.data[3] | (recvBuff.data[2] << 8);

              TargetSteeringAngle = (to16Buf / 10 + 1000) * 10;
              if (TargetSteeringAngle > AngleLimitP) {
                  TargetSteeringAngle = AngleLimitP; //470
              }
              if (TargetSteeringAngle < AngleLimitN) {
                  TargetSteeringAngle = AngleLimitN; //-470
              }

              if (spd > 0x3e8)
              {
                  if (TargetSteeringAngle > (AngleLimitP-2700)) {
                      TargetSteeringAngle = (AngleLimitP-2700); // 200
                  }
                  if (TargetSteeringAngle < (AngleLimitN+2700)) {
                      TargetSteeringAngle = (AngleLimitN+2700); // -200
                  }
              }

              //SteerByWire = RxDATA[0] >> 16;
              SteerByWire = recvBuff.data[0];

              if (TargetSteeringAngle != preTSA)
              {
                  deltaTSA = TargetSteeringAngle-preTSA;
                  // Different acc. rate for diff. delta Target
                  if ( (copysign(deltaTSA,1)> 0)|(copysign(deltaTSA,1) <= 200)) { rateTSA_acc = 10, rateTSA_dec = 10,rateTSA_spd =10 ;}
                  if ( (copysign(deltaTSA,1)> 200)|(copysign(deltaTSA,1) <= 500)) {rateTSA_acc = 20, rateTSA_dec = 15,rateTSA_spd =20;}
                  if ( (copysign(deltaTSA,1)> 500)|(copysign(deltaTSA,1) <= 2000)) { rateTSA_acc = 30, rateTSA_dec = 20,rateTSA_spd =30;}
                  if ( (copysign(deltaTSA,1 )>2000)|(copysign(deltaTSA,1)<=10000)) { rateTSA_acc = 40, rateTSA_dec = 20,rateTSA_spd = 40;}

                  //decel_limit= (RxDATA[1]>>24 & 0x000000FF)* rateTSA_dec;
                  //accel_limit= (RxDATA[1]>>16 & 0x000000FF)* rateTSA_acc;
                  //speed_limit= (RxDATA[1]>>8  & 0x000000FF)* rateTSA_spd;

					decel_limit= recvBuff.data[4] * rateTSA_dec;
					accel_limit= recvBuff.data[5] * rateTSA_acc;
					speed_limit= recvBuff.data[6] * rateTSA_spd;

                  preTSA = TargetSteeringAngle;
              }
          }
          else
          { // print out error message indicate Steering module not open!
        	//  if(uart_debug){LPUART_DRV_SendData(INST_LPUART1, (uint8_t *)"Steer closed\n", sizeof("Steer closed\n"));}
        	//  if(uart_debug){strcpy((DebugMsg2+DebugMsgSize), "2:NO by wire, Steer closed"); }
        	  if(uart_debug){strcpy((DebugMsg2), "2:NO by wire, Steer closed"); }
          }
      }


      /* Update shift control message 0x150 */
      if (recvBuff.msgId == 0x150)
      {
        /* Shift when speed = 0 */
        if (spd == 0) {
            GS_flag = 1;
            //P 0x04
            //R 0x08
            //N 0x10
            //D 0x27
            //M 0x47
            //GearShiftM = RxDATA[0] & 0x000000ff;
            GearShiftM = recvBuff.data[3];
        }
      }
}


void processCAN1(void)
{
  /* Add steering control and send to CAN2
     *  - Modify 0x3a8
     */
      if ((recvCAN1Buff.msgId == 0x3A8) && SteerByWire)
      {
    	  //CurrentSteeringAngle = RxDATA[0] & 0X7FFF;
    	  CurrentSteeringAngle = recvCAN1Buff.data[3] | ((recvCAN1Buff.data[2] & 0x7f) <<8);
    	//  if(uart_debug){LPUART_DRV_SendData(INST_LPUART1, (uint8_t *)"BY WIRE\n", sizeof("BY WIRE\n"));}
    	  if(uart_debug){strcpy((DebugMsg3), "3:Steer ByWire Enabled");}
          EnableTransition = EnableTransition << 1 | 0x1; //Track EnableTransition
          if ((EnableTransition & 0x3) == 0x01) //Enable Initial Transition from 0 to 1
          {
              ControlAngle = CurrentSteeringAngle; //initialize ControlAngle based on actual steering angle
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
          //CAN2->RAMn[ TxbufferNo * MSG_BUF_SIZE + 2] =  (RxDATA[0] & 0xFFFF0000) | 0x8000 | INT_Control_Angle;
          //CAN2->RAMn[ TxbufferNo * MSG_BUF_SIZE + 3] =  RxDATA[1];

          for (int i=0; i<8; i++) {
            sendCAN2Buff.data[i] = recvCAN1Buff.data[i];
          }

      //    sendCAN2Buff.data[0] = recvCAN1Buff.data[0] | 0x80;
     //     sendCAN2Buff.data[0] = (recvCAN1Buff.data[2] & 0x00) | 0x80;
          sendCAN2Buff.data[2] = (((INT_Control_Angle & 0xFF00) >> 8 )|0x80);
          sendCAN2Buff.data[3] =  INT_Control_Angle & 0xFF;

          SendCANDataCycleMb(INST_CANCOM2, recvCAN1Buff.msgId, sendCAN2Buff.data);
      }
      else   //relay other messages and 3A8 without steering control.
      {
          /* Relay message to CAN2 except 0x216
           *  - Modify 0x202
           *  - Modify 0x217
           *  - Modify 0x204
           */
        if (recvCAN1Buff.msgId == 0x202)
			{
			  //R_speed = RxDATA[1] & 0x0000ffff;//speed response
			  //checks = (RxDATA[0] & 0x00ff0000) >> 16;
			  //RxL = RxDATA[1] & 0x000000ff;
			  //RxH = (RxDATA[1] & 0x0000ff00) >> 8;
			  //RxLH = RxDATA[1] & 0x0000ffff;

			  checks = recvCAN1Buff.data[1];
			  RxL = recvCAN1Buff.data[7];
			  RxH = recvCAN1Buff.data[6];
			  RxLH = recvCAN1Buff.data[7] | (recvCAN1Buff.data[6]<<8);

			  spd = RxLH;

			  RxLH = RxLH >> 2;

			  RxL_n = RxLH & 0x000000ff;
			  RxH_n = (RxLH & 0x0000ff00) >> 8;

			  delta_L = RxL - RxL_n;
			  delta_H = RxH - RxH_n;

			  checks = checks + delta_L + delta_H;

			  checks = checks & 0x000000ff;

			  //RxDATA[0] = (RxDATA[0] & 0xff00ffff) | (checks << 16);
			  //RxDATA[1] = (RxDATA[1] & 0xffff0000) | RxLH;

				  for (int i=0; i<8; i++) {
					sendCAN2Buff.data[i] = recvCAN1Buff.data[i];
				  }
			  sendCAN2Buff.data[1] = checks;
			  sendCAN2Buff.data[6] = (RxLH & 0xff00) >> 8;
			  sendCAN2Buff.data[7] = (RxLH & 0xff);

			  R_speed = recvCAN1Buff.data[7] | (recvCAN1Buff.data[6]<<8);//speed response
			}
         if (recvCAN1Buff.msgId == 0x217)
          {
          /*
              RxL = RxDATA[0] & 0x0000ffff;
              RxL = RxL >> 2;
              RxH = (RxDATA[0] & 0xffff0000) >> 16;
              RxH = RxH >> 2;
              RxDATA[0] = RxL | (RxH << 16);
              */

              RxL = recvCAN1Buff.data[3] | (recvCAN1Buff.data[2]<<8);
              RxL = RxL >> 2;
              RxH = recvCAN1Buff.data[1] | (recvCAN1Buff.data[0]<<8);
              RxH = RxH >> 2;

              sendCAN2Buff.data[0] = (RxL & 0xff00) >> 8;
              sendCAN2Buff.data[1] = RxL & 0xff;
              sendCAN2Buff.data[2] = (RxH & 0xff00) >> 8;
              sendCAN2Buff.data[3] = RxH & 0xff;

              /*
              RxL = RxDATA[1] & 0x0000ffff;
              RxL = RxL >> 2;
              RxH = (RxDATA[1] & 0xffff0000) >> 16;
              RxH = RxH >> 2;
              RxDATA[1] = RxL | (RxH << 16);
              */

              RxL = recvCAN1Buff.data[7] | (recvCAN1Buff.data[6]<<8);
              RxL = RxL >> 2;
              RxH = recvCAN1Buff.data[5] | (recvCAN1Buff.data[4]<<8);
              RxH = RxH >> 2;

              sendCAN2Buff.data[4] = (RxL & 0xff00) >> 8;
              sendCAN2Buff.data[5] = RxL & 0xff;
              sendCAN2Buff.data[6] = (RxH & 0xff00) >> 8;
              sendCAN2Buff.data[7] = RxH & 0xff;
          }
         if (recvCAN1Buff.msgId == 0x204)
			{
			  /*
			  RxE = RxDATA[1] & 0x0000ffff;
			  RxE = RxE >> 2;
			  RxDATA[1] = (RxDATA[1] & 0xffff0000) | RxE;
			  */

			  RxE = recvCAN1Buff.data[7] | (recvCAN1Buff.data[6]<<8);
			  RxE = RxE >> 2;

				  for (int i=0; i<8; i++) {
					sendCAN2Buff.data[i] = recvCAN1Buff.data[i];
				  }
			  sendCAN2Buff.data[6] = (RxE & 0xff00) >> 8;
			  sendCAN2Buff.data[7] = RxE & 0xff;

			  //R_accel = RxDATA[0] & 0x03ff0000;//throttle response
			  R_accel = recvCAN1Buff.data[1] | ((recvCAN1Buff.data[0] & 0x03) << 8);
			}
        /* Execute send non-blocking */
         if (recvCAN1Buff.msgId != 0x216) // donot relay 0x216 to CAN1 (11/25)
        	{ // donot relay 0x216 to CAN1 (11/25)
              //CAN2->RAMn[ TxbufferNo * MSG_BUF_SIZE + 2] =  RxDATA[0];
              //CAN2->RAMn[ TxbufferNo * MSG_BUF_SIZE + 3] =  RxDATA[1];

				if ((recvCAN1Buff.msgId == 0x202) || (recvCAN1Buff.msgId == 0x204) || (recvCAN1Buff.msgId == 0x217)) {
				  SendCANDataCycleMb(INST_CANCOM2, recvCAN1Buff.msgId, sendCAN2Buff.data);
				} else {
					  for (int i=0; i<8; i++) {
						sendCAN2Buff.data[i] = recvCAN1Buff.data[i];
					  }
				  SendCANDataCycleMb(INST_CANCOM2, recvCAN1Buff.msgId, sendCAN2Buff.data);
				}
            }

          /* Relay response message to CAN0
           *  - Response from 0x202, 0x204, 0x7d
           *  - Response from 0x5d
           */
          if (recvCAN1Buff.msgId == 0x7D)//brake response
          	  {
					/*
				R_brake_H = RxDATA[0] & 0x0000000f;
				R_brake_L = RxDATA[1] & 0xff800000;
				*/

				R_brake_H = recvCAN1Buff.data[3] & 0x0f;
				R_brake_L = (recvCAN1Buff.data[4] << 8) | (recvCAN1Buff.data[5] & 0x80);
          	  }
          if ((recvCAN1Buff.msgId == 0x202) || (recvCAN1Buff.msgId == 0x204) || (recvCAN1Buff.msgId == 0x7D))
          	  {
				/* 0x202 R_speed */
				/* 0x204 R_accel */
				/* 0x7D  R_brake */
				/*
				RxDATA[0] = R_accel | R_speed;
				RxDATA[1] = (R_brake_H<<28)|(R_brake_L>>4);
				*/

				sendBuff.data[0] = (R_accel & 0x0300) >> 8;
				sendBuff.data[1] = R_accel & 0xff;
				sendBuff.data[2] = (R_speed & 0xff00) >> 8;
				sendBuff.data[3] = R_speed & 0xff;

				sendBuff.data[4] = (R_brake_H << 4) | ((R_brake_L & 0xf000) >> 12);
				sendBuff.data[5] = (R_brake_L & 0x0f80) >> 4;
				sendBuff.data[6] = 0;
				sendBuff.data[7] = 0;

				//FLEXCAN0_transmit_Data(RxDATA[0],RxDATA[1],0x660);
				/* Execute send non-blocking */
				SendCANDataCycleMb(INST_CANCOM0, 0x660, sendBuff.data);
          	  }
         if (recvCAN1Buff.msgId == 0x5D)
          	  {
        	  //R_shift = (RxDATA[0] & 0xE0000000)>>16;
				R_shift = recvCAN1Buff.data[0] & 0xe0;
          	  }

          /* Relay message to CAN0
           *  - ID 0x3A8
           *  - ID 0x216
           *  - ID 0x168, 0x172, 0x230
           */
         if (recvCAN1Buff.msgId == 0x3A8)//Relay 3A8 to CAN0
          	  {
              EnableTransition = EnableTransition << 1;

              for (int i=0; i<8; i++) {
                sendBuff.data[i] = recvCAN1Buff.data[i];
              }
              SendCANDataCycleMb(INST_CANCOM0, recvCAN1Buff.msgId, sendBuff.data);
          	  }
         if(recvCAN1Buff.msgId == 0x216)
          	  {
              //FLEXCAN0_transmit_Data(RxDATA[0],RxDATA[1],0x216);
              /* Execute send non-blocking */
              for (int i=0; i<8; i++) {
                sendBuff.data[i] = recvCAN1Buff.data[i];
              	  }
				SendCANDataCycleMb(INST_CANCOM0, recvCAN1Buff.msgId, sendBuff.data);

				/*
					  disR_low = (RxDATA[0] & 0xFF00) >>8;
					  disL_low =  RxDATA[0] & 0xFF;
					  */
				disR_low = recvCAN1Buff.data[2];
				disL_low = recvCAN1Buff.data[3];

              deltaR = disR_low - disR_low_t;
              deltaL = disL_low - disL_low_t;
              disR_low_t = disR_low;
              disL_low_t = disL_low;

              if (GearShift == 0x2) {//R
                  disR_low = disR_low_p - deltaR;
                  disL_low = disL_low_p - deltaL;
              	  }
              if (GearShift ==0x4){//D
                  disR_low = disR_low_p + deltaR;
                  disL_low = disL_low_p + deltaL;
              	  }
              if(disR_low_p - disR_low > 127) // 255 -> 0
                  disR_h += 1;
              if(disL_low_p - disL_low > 127)
                  disL_h += 1;
              if(disR_low - disR_low_p > 127)// 0 -> 255
                  disR_h -= 1;
              if(disL_low - disL_low_p > 127)
                  disL_h -= 1;

              /*
              disR = (disR_h<<8) | (disR_low & 0xff);
              disL = (disL_h<<8) | (disL_low & 0xff);
              */
              sendBuff.data[2] = disR_h;
              sendBuff.data[3] = disR_low & 0xff;

              sendBuff.data[6] = disL_h;
              sendBuff.data[7] = disL_low & 0xff;

              disR_low_p = disR_low;
              disL_low_p = disL_low;

              //FLEXCAN0_transmit_Data(disR,disL,0x100);
              /* Execute send non-blocking */
              SendCANDataCycleMb(INST_CANCOM0, 0x100, sendBuff.data);
          	  }
         if ((recvCAN1Buff.msgId == 0x168) || (recvCAN1Buff.msgId == 0x172) || (recvCAN1Buff.msgId == 0x230))
          	  {
              //read gear shift: P 01; R 02; N 03; D 04; L 05;
              if (recvCAN1Buff.msgId == 0x168) {
                  //GearShift = ((RxDATA[0] & 0x00ff0000) >> 16) & 0xf;
            	  GearShift = recvCAN1Buff.data[1] & 0xf;
              	  }

              //FLEXCAN0_transmit_Data(RxDATA[0],RxDATA[1],recvBuff.msgId);
              for (int i=0; i<8; i++) {
                sendBuff.data[i] = recvCAN1Buff.data[i];
              	  }
              SendCANDataCycleMb(INST_CANCOM0, recvCAN1Buff.msgId, sendBuff.data);
          }
      }
}

void processCAN2(void)
{
    /* Relay message to CAN1
     *  - Modify shift of 0x5A
     */
  if (recvCAN2Buff.msgId == 0x5A) //Gear shift
  {
        for (int i=0; i<8; i++) {
          sendCAN1Buff.data[i] = recvCAN2Buff.data[i];
        }

    if (GS_flag == 1)
    {
      //GearCS = (RxDATA[1] & 0xff000000) >> 24;
      GearCS = recvCAN2Buff.data[0];

      newCS = (GearShiftM & 0x01);
      newCS += (GearShiftM & 0x02);
      newCS += (GearShiftM & 0x04);
      newCS += (GearShiftM & 0x08)*2;

      newCS += (GearShiftM & 0x10)*3;
      newCS += (GearShiftM & 0x20)*5;
      newCS += (GearShiftM & 0x40)*9;
      newCS += (GearShiftM & 0x80)*13;

      newCS = newCS - GearCS;
      newCS = GearCS - newCS;

      if (GearShift == 0x1 || GearShift == 0x0) {
        GearCS = GearCS - 1;
        GearCSOri = (GearShiftM & 0x01);
        GearCSOri += (GearShiftM & 0x02);
        GearCSOri += (GearShiftM & 0x04);
        GearCSOri += (GearShiftM & 0x08)*2;

        GearCSOri += (GearShiftM & 0x10)*3;
        GearCSOri += (GearShiftM & 0x20)*5;
        GearCSOri += (GearShiftM & 0x40)*9;
        GearCSOri += (GearShiftM & 0x80)*13;

        GearCS = GearCS + GearCSOri;

        /*
        RxDATA[0] = (RxDATA[0] & 0xffffff00) | GearShiftM;
        RxDATA[1] = (RxDATA[1] & 0x00ffffff) | GearCS;
        RxDATA[1] = RxDATA[1] | 0x00080000;
        */

        sendCAN1Buff.data[3] = GearShiftM;
        sendCAN1Buff.data[4] = GearCS;
        sendCAN1Buff.data[5] = recvCAN2Buff.data[5] | 0x08;
      } else {
        /*
        RxDATA[0] = (RxDATA[0] & 0xffffff00) | GearShiftM;
        RxDATA[1] = (RxDATA[1] & 0x00ffffff) | newCS;
        */

        sendCAN1Buff.data[3] = GearShiftM;
        sendCAN1Buff.data[4] = newCS;
      }
      GS_flag = 0;
    }
  } else {
        for (int i=0; i<8; i++) {
          sendCAN1Buff.data[i] = recvCAN2Buff.data[i];
        }
  }
  SendCANDataCycleMb(INST_CANCOM1, recvCAN2Buff.msgId, sendCAN1Buff.data);

    /* Relay response message to CAN0
     *  - Response from 0x82, 0x85
     */
    if ((recvCAN2Buff.msgId == 0x85) || (recvCAN2Buff.msgId == 0x82))
    {
      if (recvCAN2Buff.msgId == 0x82) {
            //SteeringControlStatus = RxDATA[0] >> 14 & 0x3;
            SteeringControlStatus = (recvCAN2Buff.data[2] >> 6) & 0x3;

            if (SteeringControlStatus == 0x3 || SteeringControlStatus == 0x0) //00closed, 01open,02active,03fault
            {
                SteerByWire = 0;
            }
        }
      if (recvCAN2Buff.msgId == 0x85) {
        //R_angle = RxDATA[0] & 0xFFFF0000;
        R_angle = recvCAN2Buff.data[3] | (recvCAN2Buff.data[2] << 8);
      }
      //RxDATA[0] = R_angle | R_shift;
        sendBuff.data[0] = (R_angle & 0xff00) >> 8;
        sendBuff.data[1] = R_angle & 0xff;
        sendBuff.data[2] = R_shift & 0xe0;

    SendCANDataCycleMb(INST_CANCOM0, 0x650, sendBuff.data);
    }
}

/**
 * Manual mode interrupt handler
 */
void Brake_AutoManualISR(void)
{
  /* Check if brake pedal or accelerator pedal was pressed */
  // uint32_t pedalPressed =  PINS_DRV_GetPortIntFlag(ManualBrake_Port) & (1 << ManualBrake);
//  uint32_t pedalPressed = PINS_DRV_GetPortIntFlag(PORTA) & (1 << ManualBrake);
  pedalPressed=1;


  manualFlag = true;
  /* Clear interrupt flag */
  PINS_DRV_ClearPinIntFlagCmd(PORTA, ManualBrake);

  /* Disable auto mode by setting output low */
  PINS_DRV_ClearPins(PTD, 1 << AutoBrakeStatus);
//      PINS_DRV_ClearPins(PTA, 1 << AutoAccelStatus);
  PINS_DRV_ClearPins(PTE, 1 << AutoAccelStatus);

  dataReceived=true;


}

void ACC_AutoManualISR(void)
{
  /* Check if brake pedal or accelerator pedal was pressed */
//  uint32_t pedalPressed = PINS_DRV_GetPortIntFlag(PORTE) & (1 << ManualAccel);
//  uint32_t pedalPressed = PINS_DRV_GetPortIntFlag(PORTA) & (1 << ManualBrake);
  pedalPressed=1;

  manualFlag = true;
  /* Clear interrupt flag */
  PINS_DRV_ClearPinIntFlagCmd(PORTE, ManualAccel);


  /* Disable auto mode by setting output low */
  PINS_DRV_ClearPins(PTD, 1 << AutoBrakeStatus);
//      PINS_DRV_ClearPins(PTA, 1 << AutoAccelStatus);
  PINS_DRV_ClearPins(PTE, 1 << AutoAccelStatus);

  dataReceived=true;


}


/**
 * Input status update per 50ms handler
 */
void LPIT_ISR_50ms(void)
{
    /* Clear LPIT channel flag */
    LPIT_DRV_ClearInterruptFlagTimerChannels(INST_LPIT1, (1 << LPIT_CHANNEL0));

    /* Read manual status */
    uint32_t pedalStatus = ((PINS_DRV_ReadPins(PTA) & (1 << ManualBrake)) >> 8) | ((PINS_DRV_ReadPins(PTE)&(1 << ManualAccel)) >> 13);
//  uint32_t pedalStatus = PINS_DRV_ReadPins(PTA) & (1 << ManualBrake);

  /* Check manual status */
  if(pedalStatus != 0) {
    manualFlag = true;
    AutoControl();
  }
  	  sendBuff.data[0] = manualFlag;
  	  sendBuff.data[1] = pedalStatus;
//    sendBuff.data[2] = (PINS_DRV_GetPinsOutput(PTA) & (1 << AutoAccelStatus))>>AutoAccelStatus;
  	  sendBuff.data[2] = (PINS_DRV_GetPinsOutput(PTE) & (1 << AutoAccelStatus))>>AutoAccelStatus;
  	  sendBuff.data[3] = accelVol;
  	  sendBuff.data[6] = (PINS_DRV_GetPinsOutput(PTD) & (1 << AutoBrakeStatus))>>AutoBrakeStatus;
  	  sendBuff.data[7] = brakePWM;

  /* Execute send non-blocking */
  if(CAN0_response){ SendCANDataCycleMb(INST_CANCOM0, StateMsgID, sendBuff.data);}
//  SendCANData(INST_CANCOM1,TX_MAILBOX1, StateMsgID, sendBuff.data);
//  SendCANData(INST_CANCOM2,TX_MAILBOX2, StateMsgID, sendBuff.data);
  //LPUART_DRV_SendData(INST_LPUART1, (uint8_t *)welcomeMsg, strlen(welcomeMsg));
   // PINS_DRV_TogglePins(PTD,1 << LED2);
 //   PINS_DRV_TogglePins(PTC,1 << LED);
    //PINS_DRV_SetPins(PTC,1 << LED);
  //PINS_DRV_TogglePins(PTC,1 << LED);
  sendUART_debug();
  if(UARTCOUNT>=1)
  {
	 UARTCOUNT++;
  }
  if(UARTCOUNT==10)
  {
     UARTCOUNT=0;
  }
}

void sendUART_debug(void)
{

	    //  OnesecondISR_Toggle=1;

	if(uart_debug && (UARTCOUNT>0))
	  {
	   //LPUART_DRV_GetTransmitStatus(INST_LPUART1, &bytesRemaining) == STATUS_SUCCESS)
	   if(LPUART_DRV_GetTransmitStatus(INST_LPUART1, &bytesRemaining) == STATUS_SUCCESS)
				{
				  UART_STATUS_OPEN=true;
				switch(UARTCOUNT)
					{
					case 1:
					  {
						  LPUART_DRV_SendData(INST_LPUART1,  (uint8_t *)DebugMsg1, sizeof DebugMsg1);
					  }
					  break;
					case 2:
					  {
						  LPUART_DRV_SendData(INST_LPUART1,  (uint8_t *)DebugMsg2, sizeof DebugMsg2);
					  }
					  break;
					case 3:
					  {
						  LPUART_DRV_SendData(INST_LPUART1,  (uint8_t *)DebugMsg3, sizeof DebugMsg3);
					  }
					  break;
					case 4:
					  {

					  }
					  break;
					case 5:
					  {

					  }
					  break;
					case 6:
					  {

					  }
					  break;
					case 7:
					  {

					  }
					  break;
					case 8:
					  {

					  }
					  break;
					case 9:
					  {
						  strcpy(DebugMsg1, "1:                          \r\n");
						  strcpy(DebugMsg2, "2:                          \r\n");
						  strcpy(DebugMsg3, "3:                          \r\n");
					  }
					  break;
					default:
					  break;
					}
				}
			  else
				{
				  UART_STATUS_OPEN=false;
				}

	//	  OnesecondISR_Toggle=0;
		  //	  LPUART_DRV_SendData(INST_LPUART1,  (uint8_t *)DebugMsg1, sizeof DebugMsg1);


	//		strcpy(DebugMsg1, "1:                          \r\n2:                          \r\n3:                          \r\n");
		 }

}




void LPIT_ISR_500ms(void)
{
    /* Clear LPIT channel flag */
    LPIT_DRV_ClearInterruptFlagTimerChannels(INST_LPIT1, (1 << LPIT_CHANNEL1));


    PINS_DRV_TogglePins(PTC,1 << LED);

//    dataReceived=1; //force a autocontrol() every 500ms to update SPI and PWM
//  LPUART_DRV_SendData(INST_LPUART1, (uint8_t *)"500ms\n", sizeof("500ms\n"));

    //PINS_DRV_SetPins(PTC,1 << LED);
  //PINS_DRV_TogglePins(PTC,1 << LED);
}

void LPIT_ISR_1s(void)
{
    /* Clear LPIT channel flag */
    LPIT_DRV_ClearInterruptFlagTimerChannels(INST_LPIT1, (1 << LPIT_CHANNEL2));
    OnesecondISR_Toggle=1;
    UARTCOUNT=1; // start(reset) uart sequence every second.
//
//    if(uart_debug){
//	  LPUART_DRV_SendDataBlocking(INST_LPUART1, (uint8_t *)"1s\n", sizeof("1s\n"),10);
//
//	  LPUART_DRV_SendDataBlocking(INST_LPUART1,  (uint8_t *)DebugMsg1, sizeof DebugMsg1,10);
//	  LPUART_DRV_SendDataBlocking(INST_LPUART1,  (uint8_t *)DebugMsg2, sizeof DebugMsg2,10);
//	  LPUART_DRV_SendDataBlocking(INST_LPUART1,  DebugMsg3, sizeof DebugMsg3,10);
//	  LPUART_DRV_SendDataBlocking(INST_LPUART1,  DebugMsg4, sizeof DebugMsg4,10);
//	  LPUART_DRV_SendDataBlocking(INST_LPUART1,  DebugMsg5, sizeof DebugMsg5,10);


//  }//timeout asumed to be 1ms
  //LPUART_DRV_SetTxBuffer(INST_LPUART1, (uint8_t *)"1sec\n", sizeof("1sec\n"));
  //UpdateSPI(volAl, cs1);
  //UpdateSPI(volBl, cs2);
  //refreshSPI=1;  //goes to main loop and refresh spi every second.




}
void LPIT_ISR_2s(void)
{
    /* Clear LPIT channel flag */
    LPIT_DRV_ClearInterruptFlagTimerChannels(INST_LPIT1, (1 << LPIT_CHANNEL3));
//  LPUART_DRV_SendData(INST_LPUART1, (uint8_t *)uart_buffer, UART_BUFFER_SIZE);
//  if(LPUART_DRV_GetTransmitStatus==STATUS_SUCCESS)
//   LPUART_DRV_SendData(INST_LPUART1, (uint8_t *)uart_buffer, 20);
  //LPUART_DRV_SendDataBlocking(INST_LPUART1, (uint8_t *)"2sec\n", sizeof("2sec\n"),100U);//timeout asumed to be 1ms
  //LPUART_DRV_SetTxBuffer(INST_LPUART1, (uint8_t *)"2sec\n", sizeof("2sec\n"));
}
volatile int exit_code = 0;
/* User includes (#include below this line is not maintained by Processor Expert) */

/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - __start (startup asm routine)
 * - __init_hardware()
 * - main()
 *   - PE_low_level_init()
 *     - Common_Init()
 *     - Peripherals_Init()
*/
int main(void)
{
  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                 /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/

    /* Do the initializations required for this application */
  BoardInit();
  GPIOInit();
  LPUARTInit();
  FlexCANInit();
  PWMInit();
  LPTMRInit();
  LPSPInit();
    while(1)
    {

  //    PINS_DRV_TogglePins(PTD,1 << LED2);
  //    PINS_DRV_TogglePins(PTC,1 << LED);
//      while (n<10000000)
//      {
//        n++;
//      }
//      n=0;

//      if(CAN0Received){
//        processCAN0();
 //       CAN0Received = 0;
//      }

      if(dataReceived)
      {
        AutoControl();
        dataReceived=false;
      }

      if(refreshSPI)
      {
        UpdateSPI(voltageA, cs1);
        UpdateSPI(voltageB, cs2);
        refreshSPI=0;
      }
      //  PINS_DRV_TogglePins(PTE,1 << 4);
  //    sendUART_debug();
    }

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
