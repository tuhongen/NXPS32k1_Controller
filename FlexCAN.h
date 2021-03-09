/* FlexCAN.h              (c) 2015 Freescale Semiconductor, Inc.
 * Descriptions: FTM example code.
 * 16 Sep 2016 SM: Initial version
 */


#ifndef FLEXCAN_H_
#define FLEXCAN_H_

#define NODE_A        /* If using 2 boards as 2 nodes, NODE A & B use different CAN IDs */

void FLEXCAN0_init (void);
void FLEXCAN1_init (void);
void FLEXCAN2_init (void);
void FLEXCAN0_transmit_Data(uint32_t Data1,uint32_t Data2,uint32_t ID);
void FLEXCAN1_transmit_Data(uint32_t Data1,uint32_t Data2,uint32_t ID);
void FLEXCAN2_transmit_Data(uint32_t Data1,uint32_t Data2,uint32_t ID);
void FLEXCAN0_transmit_msg (void);
void FLEXCAN0_receive_msg (void);
void FLEXCAN1_transmit_msg (void);
void FLEXCAN2_transmit_msg (void);

#endif /* FLEXCAN_H_ */
