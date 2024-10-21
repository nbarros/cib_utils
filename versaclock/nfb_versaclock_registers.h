/*
 * nfb_versaclock_registers.h
 *
 *  Created on: Oct 15, 2023
 *      Author: nbarros
 */

#ifndef ZYNQMP_FSBL_NFB_VERSACLOCK_REGISTERS_H_
#define ZYNQMP_FSBL_NFB_VERSACLOCK_REGISTERS_H_

#ifndef VS_STRUCT
#define VS_STRUCT
typedef struct versaclock_register
{
   uint8_t size;
   uint8_t addr;
   uint8_t val;
} versaclock_register;
#endif


#ifdef NFB_CIB_VC5_CONFIG_0

//FIXME: correct for the right number of registers
#define VC5_NUM_REGS 102

versaclock_register vc5_reg_store[VC5_NUM_REGS] =
{
 {0x1, 0x00, 0x61}, // keep the address at 0xD4
 {0x1, 0x01, 0xFF},
 {0x1, 0x02, 0x00},
 {0x1, 0x03, 0x00},
 {0x1, 0x04, 0x00},
 {0x1, 0x05, 0x00},
 {0x1, 0x06, 0x00},
 {0x1, 0x07, 0x00},
 {0x1, 0x08, 0x00},
 {0x1, 0x09, 0xFF},
 {0x1, 0x0A, 0x01},
 {0x1, 0x0B, 0x00},
 {0x1, 0x0C, 0x00},
 {0x1, 0x0D, 0xB6},
 {0x1, 0x0E, 0xB4},
 {0x1, 0x0F, 0x92},  // the previous were the default trimmings
 {0x1, 0x10, 0x80},
 {0x1, 0x11, 0x0C},
 {0x1, 0x12, 0x01},
 {0x1, 0x13, 0x00},
 {0x1, 0x14, 0x00},
 {0x1, 0x15, 0x00},
 {0x1, 0x16, 0x8C},
 {0x1, 0x17, 0x07},
 {0x1, 0x18, 0x00},
 {0x1, 0x19, 0x00},
 {0x1, 0x1A, 0x00},
 {0x1, 0x1B, 0x00},
 {0x1, 0x1C, 0x85},
 {0x1, 0x1D, 0x8F},
 {0x1, 0x1E, 0xBA},
 {0x1, 0x1F, 0x32}, // 0x20 is factory reserved
 {0x1, 0x21, 0x81},
 {0x1, 0x22, 0x00},
 {0x1, 0x23, 0x00},
 {0x1, 0x24, 0x00},
 {0x1, 0x25, 0x00},
 {0x1, 0x26, 0x00},
 {0x1, 0x27, 0x00},
 {0x1, 0x28, 0x00},
 {0x1, 0x29, 0x00},
 {0x1, 0x2A, 0x00},
 {0x1, 0x2B, 0x00},
 {0x1, 0x2C, 0x00},
 {0x1, 0x2D, 0x00},
 {0x1, 0x2E, 0xE0},
 {0x1, 0x2F, 0x00}, // 0x30 is factory reserved
 {0x1, 0x31, 0x81},
 {0x1, 0x32, 0x00},
 {0x1, 0x33, 0x00},
 {0x1, 0x34, 0x00},
 {0x1, 0x35, 0x00},
 {0x1, 0x36, 0x00},
 {0x1, 0x37, 0x00},
 {0x1, 0x38, 0x00},
 {0x1, 0x39, 0x00},
 {0x1, 0x3A, 0x00},
 {0x1, 0x3B, 0x00},
 {0x1, 0x3C, 0x00},
 {0x1, 0x3D, 0x00},
 {0x1, 0x3E, 0xE0},
 {0x1, 0x3F, 0x00}, // 0x40 is factory reserved
 {0x1, 0x41, 0x81},
 {0x1, 0x42, 0x00},
 {0x1, 0x43, 0x00},
 {0x1, 0x44, 0x00},
 {0x1, 0x45, 0x00},
 {0x1, 0x46, 0x00},
 {0x1, 0x47, 0x00},
 {0x1, 0x48, 0x00},
 {0x1, 0x49, 0x00},
 {0x1, 0x4A, 0x00},
 {0x1, 0x4B, 0x00},
 {0x1, 0x4C, 0x00},
 {0x1, 0x4D, 0x00},
 {0x1, 0x4E, 0xE0},
 {0x1, 0x4F, 0x00}, // 0x50 is factory reserved
 {0x1, 0x51, 0x81},
 {0x1, 0x52, 0x00},
 {0x1, 0x53, 0x00},
 {0x1, 0x54, 0x00},
 {0x1, 0x55, 0x00},
 {0x1, 0x56, 0x00},
 {0x1, 0x57, 0x00},
 {0x1, 0x58, 0x00},
 {0x1, 0x59, 0x00},
 {0x1, 0x5A, 0x00},
 {0x1, 0x5B, 0x00},
 {0x1, 0x5C, 0x00},
 {0x1, 0x5D, 0x00},
 {0x1, 0x5E, 0xE0},
 {0x1, 0x5F, 0x00},
 {0x1, 0x60, 0x63}, // conf CLK 1
 {0x1, 0x61, 0x01}, // enable CLK 1
 {0x1, 0x62, 0x63}, // conf CLK 2
 {0x1, 0x63, 0x01}, // enable CLK 2
 {0x1, 0x64, 0x63}, // conf CLK 3
 {0x1, 0x65, 0x01}, // enable CLK 3
 {0x1, 0x66, 0x63}, // conf CLK 4
 {0x1, 0x67, 0x01}, // enable CLK 4
 {0x1, 0x68, 0xFC}, // OE_SHDN
 {0x1, 0x69, 0xFC}  // OS_SHDN
};


#elif defined(NFB_CIB_VC5_CONFIG_1)

#define VC5_NUM_REGS 102

versaclock_register vc5_reg_store[VC5_NUM_REGS] =
{
 {0x1, 0x00, 0x61}, // keep the address at 0xD4
 {0x1, 0x01, 0xFF},
 {0x1, 0x02, 0x00},
 {0x1, 0x03, 0x00},
 {0x1, 0x04, 0x00},
 {0x1, 0x05, 0x00},
 {0x1, 0x06, 0x00},
 {0x1, 0x07, 0x00},
 {0x1, 0x08, 0x00},
 {0x1, 0x09, 0xFF},
 {0x1, 0x0A, 0x01},
 {0x1, 0x0B, 0x00},
 {0x1, 0x0C, 0x00},
 {0x1, 0x0D, 0xB6},
 {0x1, 0x0E, 0xB4},
 {0x1, 0x0F, 0x92},  // the previous were the default trimmings
 {0x1, 0x10, 0x80},
 {0x1, 0x11, 0x0C},
 {0x1, 0x12, 0x81}, // cap 1
 {0x1, 0x13, 0x80}, // cap 2
 {0x1, 0x14, 0x00},
 {0x1, 0x15, 0x03}, // ref_divider
 {0x1, 0x16, 0x8C},
 {0x1, 0x17, 0x06}, // feedback divider
 {0x1, 0x18, 0x40}, // feedback divider
 {0x1, 0x19, 0x00}, 
 {0x1, 0x1A, 0x00},
 {0x1, 0x1B, 0x00},
 {0x1, 0x1C, 0x9F}, // reserved bits
 {0x1, 0x1D, 0xFF}, // reserved bits 
 {0x1, 0x1E, 0xD1}, // reserved bits 
 {0x1, 0x1F, 0x3C}, // reserved bits // 0x20 is factory reserved
 {0x1, 0x21, 0x81},
 {0x1, 0x22, 0x00},
 {0x1, 0x23, 0x00},
 {0x1, 0x24, 0x00},
 {0x1, 0x25, 0x00},
 {0x1, 0x26, 0x00},
 {0x1, 0x27, 0x00},
 {0x1, 0x28, 0x00},
 {0x1, 0x29, 0x00},
 {0x1, 0x2A, 0x04}, // reserved bits
 {0x1, 0x2B, 0x00}, 
 {0x1, 0x2C, 0x00},
 {0x1, 0x2D, 0x00},
 {0x1, 0x2E, 0x40}, // OD1 integer 0
 {0x1, 0x2F, 0x00}, // 0x30 is factory reserved
 {0x1, 0x31, 0x81},
 {0x1, 0x32, 0x00},
 {0x1, 0x33, 0x04}, // OD2 fraction
 {0x1, 0x34, 0x97}, // OD2 fraction
 {0x1, 0x35, 0xB4}, // OD2 fraction
 {0x1, 0x36, 0x00}, 
 {0x1, 0x37, 0x00},
 {0x1, 0x38, 0x00},
 {0x1, 0x39, 0x00},
 {0x1, 0x3A, 0x04}, // reserved bits
 {0x1, 0x3B, 0x00},
 {0x1, 0x3C, 0x00},
 {0x1, 0x3D, 0x00},
 {0x1, 0x3E, 0x40}, // OD2 integer
 {0x1, 0x3F, 0x00}, // 0x40 is factory reserved
 {0x1, 0x41, 0x81},
 {0x1, 0x42, 0x00},
 {0x1, 0x43, 0x04}, // OD3 fraction
 {0x1, 0x44, 0x97}, // OD3 fraction
 {0x1, 0x45, 0xB4}, // OD3 fraction
 {0x1, 0x46, 0x00},
 {0x1, 0x47, 0x00},
 {0x1, 0x48, 0x00},
 {0x1, 0x49, 0x00},
 {0x1, 0x4A, 0x04}, // reserved bit
 {0x1, 0x4B, 0x00},
 {0x1, 0x4C, 0x00},
 {0x1, 0x4D, 0x00},
 {0x1, 0x4E, 0x40}, // OD3 integer
 {0x1, 0x4F, 0x00}, // 0x50 is factory reserved
 {0x1, 0x51, 0x81},
 {0x1, 0x52, 0x00},
 {0x1, 0x53, 0x04}, // OD4 fraction
 {0x1, 0x54, 0x97}, // OD4 fraction
 {0x1, 0x55, 0xB4}, // OD4 fraction
 {0x1, 0x56, 0x00},
 {0x1, 0x57, 0x00},
 {0x1, 0x58, 0x00},
 {0x1, 0x59, 0x00},
 {0x1, 0x5A, 0x04}, // reserved bit
 {0x1, 0x5B, 0x00},
 {0x1, 0x5C, 0x00},
 {0x1, 0x5D, 0x00},
 {0x1, 0x5E, 0x40}, // OD4 integer
 {0x1, 0x5F, 0x00},
 {0x1, 0x60, 0x63}, // conf CLK 1
 {0x1, 0x61, 0x01}, // enable CLK 1
 {0x1, 0x62, 0x63}, // conf CLK 2
 {0x1, 0x63, 0x01}, // enable CLK 2
 {0x1, 0x64, 0x63}, // conf CLK 3
 {0x1, 0x65, 0x01}, // enable CLK 3
 {0x1, 0x66, 0x63}, // conf CLK 4
 {0x1, 0x67, 0x01}, // enable CLK 4
 {0x1, 0x68, 0xFC}, // OE_SHDN
 {0x1, 0x69, 0xFC}  // OS_SHDN
};

//
//
// -- configurations below this point are not fully functional
//
//
#elif defined(NFB_CIB_VC5_CONFIG_1_ALT)

#define VC5_NUM_REGS 42

versaclock_register const vc5_reg_store[VC5_NUM_REGS] =
{
 {0x1, 0x00, 0x60},
 {0x1, 0x10, 0x84},
 {0x1, 0x11, 0x0C},
 {0x1, 0x12, 0x81},
 {0x1, 0x13, 0x80},
 {0x1, 0x14, 0x00},
 {0x1, 0x15, 0x03},
 {0x1, 0x16, 0x8C},
 {0x1, 0x17, 0x06},
 {0x1, 0x18, 0x60},
 {0x1, 0x19, 0x00},
 {0x1, 0x1A, 0x00},
 {0x1, 0x1B, 0x00},
 {0x1, 0x1C, 0x9F},
 {0x1, 0x1D, 0xFF},
 {0x1, 0x1E, 0xE0},
 {0x1, 0x1F, 0x80},
 {0x1, 0x21, 0x0C},
 {0x1, 0x22, 0x00},
 {0x1, 0x23, 0x00},
 {0x1, 0x24, 0x00},
 {0x1, 0x25, 0x00},
 {0x1, 0x26, 0x00},
 {0x1, 0x27, 0x00},
 {0x1, 0x28, 0x00},
 {0x1, 0x29, 0x00},
 {0x1, 0x2A, 0x04},
 {0x1, 0x2B, 0x00},
 {0x1, 0x2C, 0x00},
 {0x1, 0x2D, 0x00},
 {0x1, 0x2E, 0x00},
 {0x1, 0x2F, 0x00},
 {0x1, 0x60, 0x63},
 {0x1, 0x61, 0x01},
 {0x1, 0x62, 0x63},
 {0x1, 0x63, 0x01},
 {0x1, 0x64, 0x63},
 {0x1, 0x65, 0x01},
 {0x1, 0x66, 0x63},
 {0x1, 0x67, 0x01},
 {0x1, 0x68, 0xFC}, // OE_SHDN
 {0x1, 0x69, 0x04}  // OS_SHDN
};

#elif defined(NFB_CIB_VC5_CONFIG_TEST)

#define VC5_NUM_REGS 106

versaclock_register const vc5_reg_store_untrimmed[VC5_NUM_REGS] =
{
 {0x1, 0x00, 0x60},
 // range 0x01 - 0x0f is factory reserved
 {0x1, 0x01, 0xFF},
 {0x1, 0x02, 0x00},
 {0x1, 0x03, 0x00},
 {0x1, 0x04, 0x00},
 {0x1, 0x05, 0x00},
 {0x1, 0x06, 0x00},
 {0x1, 0x07, 0x00},
 {0x1, 0x08, 0x00},
 {0x1, 0x09, 0xFF},
 {0x1, 0x0A, 0x01},
 {0x1, 0x0B, 0x00},
 {0x1, 0x0C, 0x00},
 {0x1, 0x0D, 0xB6},
 {0x1, 0x0E, 0xB4},
 {0x1, 0x0F, 0x92},
 {0x1, 0x10, 0x84},
 {0x1, 0x11, 0x0C},
 {0x1, 0x12, 0x81},
 {0x1, 0x13, 0x80},
 {0x1, 0x14, 0x00},
 {0x1, 0x15, 0x03},
 {0x1, 0x16, 0x8C},
 {0x1, 0x17, 0x06},
 {0x1, 0x18, 0x60},
 {0x1, 0x19, 0x00},
 {0x1, 0x1A, 0x00},
 {0x1, 0x1B, 0x00},
 {0x1, 0x1C, 0x9F},
 {0x1, 0x1D, 0xFF},
 {0x1, 0x1E, 0xE0},
 {0x1, 0x1F, 0x80},
 // register 0x20 is factory reserved
 {0x1, 0x20, 0x00},
 {0x1, 0x21, 0x0C},
 {0x1, 0x22, 0x00},
 {0x1, 0x23, 0x00},
 {0x1, 0x24, 0x00},
 {0x1, 0x25, 0x00},
 {0x1, 0x26, 0x00},
 {0x1, 0x27, 0x00},
 {0x1, 0x28, 0x00},
 {0x1, 0x29, 0x00},
 {0x1, 0x2A, 0x04},
 {0x1, 0x2B, 0x00},
 {0x1, 0x2C, 0x00},
 {0x1, 0x2D, 0x00},
 {0x1, 0x2E, 0x00},
 {0x1, 0x2F, 0x00},
 /* Registers 0x30, 0x40, 0x50 are factory reserved */
 {0x1, 0x30, 0x00},
 {0x1, 0x31, 0x81},
 {0x1, 0x32, 0x00},
 {0x1, 0x33, 0x00},
 {0x1, 0x34, 0x00},
 {0x1, 0x35, 0x00},
 {0x1, 0x36, 0x00},
 {0x1, 0x37, 0x00},
 {0x1, 0x38, 0x00},
 {0x1, 0x39, 0x00},
 {0x1, 0x3A, 0x04},
 {0x1, 0x3B, 0x00},
 {0x1, 0x3C, 0x00},
 {0x1, 0x3D, 0x00},
 {0x1, 0x3E, 0x00},
 {0x1, 0x3F, 0x00},
 {0x1, 0x40, 0x00},
 {0x1, 0x41, 0x81},
 {0x1, 0x42, 0x00},
 {0x1, 0x43, 0x00},
 {0x1, 0x44, 0x00},
 {0x1, 0x45, 0x00},
 {0x1, 0x46, 0x00},
 {0x1, 0x47, 0x00},
 {0x1, 0x48, 0x00},
 {0x1, 0x49, 0x00},
 {0x1, 0x4A, 0x04},
 {0x1, 0x4B, 0x00},
 {0x1, 0x4C, 0x00},
 {0x1, 0x4D, 0x00},
 {0x1, 0x4E, 0x00},
 {0x1, 0x4F, 0x00},
 {0x1, 0x50, 0x00},
 {0x1, 0x51, 0x81},
 {0x1, 0x52, 0x00},
 {0x1, 0x53, 0x00},
 {0x1, 0x54, 0x00},
 {0x1, 0x55, 0x00},
 {0x1, 0x56, 0x00},
 {0x1, 0x57, 0x00},
 {0x1, 0x58, 0x00},
 {0x1, 0x59, 0x00},
 {0x1, 0x5A, 0x04},
 {0x1, 0x5B, 0x00},
 {0x1, 0x5C, 0x00},
 {0x1, 0x5D, 0x00},
 {0x1, 0x5E, 0x00},
 {0x1, 0x5F, 0x00},
 {0x1, 0x60, 0x63},
 {0x1, 0x61, 0x01},
 {0x1, 0x62, 0x63},
 {0x1, 0x63, 0x01},
 {0x1, 0x64, 0x63},
 {0x1, 0x65, 0x01},
 {0x1, 0x66, 0x63},
 {0x1, 0x67, 0x01},
 {0x1, 0x68, 0xFC},
 {0x1, 0x69, 0x04}
};

#elif defined(NFB_CIB_VC5_CONFIG_RAW)

#define VC5_NUM_REGS 91
versaclock_register const vc5_reg_store[VC5_NUM_REGS] = {
{0x1, 0x78, 0x01},
{0x1, 0x77, 0x01},
{0x1, 0x73, 0x00},
{0x1, 0x74, 0x00},
{0x1, 0x75, 0x00},
{0x1, 0x76, 0x00},
{0x1, 0x72, 0x01},
{0x1, 0x72, 0x00},
{0x1, 0x00, 0x00},
{0x1, 0x01, 0x00},
{0x1, 0x78, 0x01},
{0x1, 0x77, 0x01},
{0x1, 0x73, 0x0C},
{0x1, 0x74, 0x80},
{0x1, 0x75, 0x0C},
{0x1, 0x76, 0x80},
{0x1, 0x72, 0x01},
{0x1, 0x72, 0x00},
{0x1, 0x78, 0x01},
{0x1, 0x77, 0x01},
{0x1, 0x73, 0x39},
{0x1, 0x74, 0x80},
{0x1, 0x75, 0x39},
{0x1, 0x76, 0x80},
{0x1, 0x72, 0x01},
{0x1, 0x72, 0x00},
{0x1, 0x78, 0x01},
{0x1, 0x77, 0x01},
{0x1, 0x73, 0x66},
{0x1, 0x74, 0x80},
{0x1, 0x75, 0x66},
{0x1, 0x76, 0x80},
{0x1, 0x72, 0x01},
{0x1, 0x72, 0x00},
{0x1, 0x78, 0x01},
{0x1, 0x77, 0x01},
{0x1, 0x73, 0x93},
{0x1, 0x74, 0x80},
{0x1, 0x75, 0x93},
{0x1, 0x76, 0x80},
{0x1, 0x72, 0x01},
{0x1, 0x72, 0x00},
{0x1, 0x01, 0x00},
{0x1, 0x78, 0x01},
{0x1, 0x77, 0x01},
{0x1, 0x73, 0x00},
{0x1, 0x74, 0x80},
{0x1, 0x75, 0x00},
{0x1, 0x76, 0x80},
{0x1, 0x72, 0x01},
{0x1, 0x72, 0x00},
{0x1, 0x78, 0x01},
{0x1, 0x77, 0x01},
{0x1, 0x73, 0x0C},
{0x1, 0x74, 0x80},
{0x1, 0x75, 0x0C},
{0x1, 0x76, 0x80},
{0x1, 0x72, 0x01},
{0x1, 0x72, 0x00},
{0x1, 0x78, 0x01},
{0x1, 0x77, 0x01},
{0x1, 0x73, 0x39},
{0x1, 0x74, 0x80},
{0x1, 0x75, 0x39},
{0x1, 0x76, 0x80},
{0x1, 0x72, 0x01},
{0x1, 0x72, 0x00},
{0x1, 0x78, 0x01},
{0x1, 0x77, 0x01},
{0x1, 0x73, 0x66},
{0x1, 0x74, 0x80},
{0x1, 0x75, 0x66},
{0x1, 0x76, 0x80},
{0x1, 0x72, 0x01},
{0x1, 0x72, 0x00},
{0x1, 0x78, 0x01},
{0x1, 0x77, 0x01},
{0x1, 0x73, 0x93},
{0x1, 0x74, 0x80},
{0x1, 0x75, 0x93},
{0x1, 0x76, 0x80},
{0x1, 0x72, 0x01},
{0x1, 0x72, 0x00},
{0x1, 0x01, 0x00},
{0x5, 0x6A, 0x0000000000},
{0x20, 0x00, 0x010F00000000000000FF01C000B6B492440C818000038C01900000009FFFE080},
{0x20, 0x20, 0x000C000000000000000004000000000000810000000000000000040000000000},
{0x20, 0x40, 0x0081000000000000000004000000000000810000000000000000040000000000},
{0xA, 0x60, 0x6301630163016301FC04},
{0x1, 0x76, 0x00},
{0x1, 0x76, 0x20}
};

#endif /*VC5_CONFIGS*/

#endif /* ZYNQMP_FSBL_NFB_VERSACLOCK_REGISTERS_H_ */
