/*
 * cib_mem.h
 *
 *  Created on: May 11, 2024
 *      Author: Nuno Barros
 */

#ifndef COMMON_CIB_MEM_H_
#define COMMON_CIB_MEM_H_

// reg 0 ised for reset
// this is no longer used. It was buggy
//#define CONF_MEM_LOW   0x00A0070000
//#define CONF_MEM_HIGH  0x00A007FFFF
//#define CONF_CH_OFFSET 0x4

#define PDTS_REG    0
#define I_0_REG     1
#define I_1_REG     2
#define ALIGN_REG   3
#define LASER_REG   4
#define MISC_REG    5
#define MOTOR_1_REG 6
#define MOTOR_2_REG 7
#define MOTOR_3_REG 8

#define GPIO_PDTS_MEM_LOW   0xA0010000
#define GPIO_PDTS_MEM_HIGH  0xA001FFFF

#define GPIO_I_0_MEM_LOW   0xA0020000
#define GPIO_I_0_MEM_HIGH  0xA002FFFF

#define GPIO_I_1_MEM_LOW   0xA0030000
#define GPIO_I_1_MEM_HIGH  0xA003FFFF

#define GPIO_ALIGN_MEM_LOW 0x00A0040000
#define GPIO_ALIGN_MEM_HIGH 0x00A004FFFF

#define GPIO_LASER_MEM_LOW   0xA0060000
#define GPIO_LASER_MEM_HIGH  0xA006FFFF

#define GPIO_MISC_MEM_LOW   0x00A0080000
#define GPIO_MISC_MEM_HIGH  0x00A008FFFF

#define GPIO_MOTOR_1_MEM_LOW   0x00A0090000
#define GPIO_MOTOR_1_MEM_HIGH  0x00A0090FFF

#define GPIO_MOTOR_2_MEM_LOW   0x00A00A0000
#define GPIO_MOTOR_2_MEM_HIGH  0x00A00A0FFF

#define GPIO_MOTOR_3_MEM_LOW   0x00A00B0000
#define GPIO_MOTOR_3_MEM_HIGH  0x00A00B0FFF

#define GPIO_CH_OFFSET      0x8

#define AXIS_FIFO_DAQ  0x00A0000000
#define AXIS_FIFO_DAQ_DEV "/dev/axis_fifo_0x00000000a0000000"

#define AXIS_FIFO_PTR  0x00A0050000
#define AXIS_FIFO_PTR_DEV "/dev/axis_fifo_0x00000000a0050000"

#define AXIS_FIFO_LOG  0x00A0070000
#define AXIS_FIFO_LOG_DEV "/dev/axis_fifo_0x00000000a0070000"


#define PAGE_SIZE     4096

#endif /* COMMON_CIB_MEM_H_ */
