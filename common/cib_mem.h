/*
 * cib_mem.h
 *
 *  Created on: May 11, 2024
 *      Author: Nuno Barros
 */

#ifndef COMMON_CIB_MEM_H_
#define COMMON_CIB_MEM_H_


// reg 0 ised for reset
#define CONF_MEM_LOW   0x0080000000
#define CONF_MEM_HIGH  0x008000FFFF
#define CONF_CH_OFFSET 0x4

#define GPIO_PDTS_MEM_LOW   0xA0010000
#define GPIO_PDTS_MEM_HIGH  0xA001FFFF

#define GPIO_I_0_MEM_LOW   0xA0020000
#define GPIO_I_0_MEM_HIGH  0xA002FFFF

#define GPIO_I_1_MEM_LOW   0xA0030000
#define GPIO_I_1_MEM_HIGH  0xA003FFFF

#define GPIO_ALIGN_MEM_LOW 0x00A0040000
#define GPIO_ALIGN_MEM_HIGH 0x00A004FFFF
#define GPIO_CH_OFFSET      0x8



#endif /* COMMON_CIB_MEM_H_ */
