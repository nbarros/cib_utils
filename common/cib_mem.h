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
#define AXIS_FIFO_PTR  0x00A0050000

#define PAGE_SIZE     4096

typedef struct daq_trigger_t
{
  // lsb
  uint32_t pos_m3 : 17;
  uint32_t pos_m2_lsb : 15;
  uint32_t pos_m2_msb : 7;
  uint32_t pos_m1 : 22;
  uint64_t timestamp;
  // msb
  static const uint32_t mask_m3 = 0x1FFFF;
  static const uint32_t mask_m2_1 = 0xFFFE0000;
  static const uint32_t mask_m2_2 = 0x1FFFF;
  static const uint32_t mask_m1 = 0x1FFFF;
  //int32_t get_pos_m1() {return cib::util::cast_to_signed(pos_m1,mask_m1);}
} daq_trigger_t;

#endif /* COMMON_CIB_MEM_H_ */
