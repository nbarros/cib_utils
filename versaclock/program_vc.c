/*
 * program_vc.c
 *
 *  Created on: Nov 29, 2023
 *      Author: nbarros
 */


/*
 * Simple I2C example
 *
 * Copyright 2017 Joel Stanley <joel@jms.id.au>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <err.h>
#include <errno.h>
#include <unistd.h>

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>



/// SOME VC5 defines
#define VC5_SRC_STATUS_REGISTER   0x9D
#define VC5_LOS_BOTH        BIT(0)
#define VC5_LOS_LOCK        BIT(1)
#define VC5_LOS_CLKIN       BIT(2)
#define VC5_LOS_PRI_IN      BIT(3)
#define VC5_LOS_ALT_IN      BIT(4)
#define VC5_LOS_ANY         VC5_LOS_ALT_IN | VC5_LOS_PRI_IN | VC5_LOS_CLKIN | VC5_LOS_LOCK | VC5_LOS_BOTH
#define VC5_LOS_MASK        0x1F

#define VC5_STATUS_REGISTER_GLOBAL   0x9F
#define VC5_STATUS_INIT_DONE        BIT(7)
#define VC5_STATUS_INIT_ACTIVE      BIT(3)
#define VC5_STATUS_MARGIN_RD_FAIL   BIT(1)
#define VC5_STATUS_MASK             BIT(7) | BIT(3) | BIT(1)


/**
 * VersaClock specific setup
 */

#define BIT(n) (1UL << n)

/*
typedef struct versaclock_register
{
   uint8_t size;
   uint8_t addr;
   uint8_t val;
} versaclock_register;
*/


//max delay for calibration from SI documentation 300ms
#define VC5_TIME_CHECK_PLL_CONFIG_US  0x50000U
//delay
// 131 ms (need to be at least 100 ms)
#define VC5_DELAY_AFTER_PLL_CONFIG_US 0x20000U
#define NFB_CIB_VC5_CONFIG_1

#include <nfb_versaclock_registers.h>

const uint8_t reset_reg = 0x76;
const uint8_t reset_mask = BIT(5);

int i2c_bus_number = 6;
int i2c_dev_number = 0x6a;
uint8_t chip_addr = 0x6a;
int fd = 0x0;
/**
 * Open I2C device
 */


static inline int open_i2c_device ( int i2c_bus_number , int i2c_device_number )
{
  char filename[12] ;
  snprintf ( filename , 11 , "/dev/i2c-%i", i2c_bus_number) ;
  int file_descriptor = open ( filename, O_RDWR ) ;
  if ( file_descriptor < 0 )
  {
    printf ( "failed to open %s, open() returned %i\n" , filename , file_descriptor ) ;
  }
  if ( ioctl ( file_descriptor , I2C_SLAVE, i2c_device_number ) < 0 )
  {
    printf ( "failed to find device %i on i2c bus %i\n" , i2c_device_number , i2c_bus_number ) ;
  }
  return ( file_descriptor ) ;
}

static inline void i2c_write_register ( int file_descriptor , uint8_t register_to_write_to , uint8_t data_to_write_to_register )
{
  uint8_t message[2] ;
  message[0] = register_to_write_to ;
  message[1] = data_to_write_to_register ;
  // we expect to write 2 bytes (address + content)
  int i = write ( file_descriptor , message , 2 ) ;
  if ( i != 2 )
  {
    printf ( "error: i2c write returned %i instead of 2\n" , i ) ;
  }
}

static inline uint8_t i2c_read_register( int file_descriptor , uint8_t register_to_read_from )
{
  uint8_t message[1] ;
  message[0] = register_to_read_from ;
  int i = write ( file_descriptor , message , 1 ) ;
  if ( i != 1 )
  {
    printf ( "error: i2c write returned %i instead of 1\n" , i ) ;
  }
  i = read ( file_descriptor , message , 1 ) ;
  if ( i != 1 )
  {
    printf ( "error: i2c read returned %i instead of 1\n" , i ) ;
  }
  return ( message[0] ) ;
}

static inline void i2c_write_register_mask ( int file_descriptor , uint8_t register_to_write_to , uint8_t data_to_write_to_register, uint8_t mask)
{

  // since there is a mask, this is a two step process.
  // 1. Read the existing contents of the register
  uint8_t cache = i2c_read_register(file_descriptor,register_to_write_to);

  // 2. compute the data with the mask
  uint8_t message  = (cache & ~mask ) | (data_to_write_to_register & mask);

  // Write the updated data to the register
  i2c_write_register(file_descriptor,register_to_write_to,message);
}


#if defined(USE_SMBUS_API)
  static inline int32_t i2c_smbus_access(int file, char read_write, uint8_t command, int size, union i2c_smbus_data *data)
  {
    struct i2c_smbus_ioctl_data args;

    args.read_write = read_write;
    args.command = command;
    args.size = size;
    args.data = data;
    return ioctl(file,I2C_SMBUS,&args);
  }


  static inline int32_t i2c_smbus_read_byte_data(int file, uint8_t command)
  {
    union i2c_smbus_data data;
    if (i2c_smbus_access(file,I2C_SMBUS_READ,command,
                         I2C_SMBUS_BYTE_DATA,&data))
      return -1;
    else
      return 0x0FF & data.byte;
  }

  static inline int32_t i2c_smbus_write_byte_data(int file, uint8_t command)
  {
    union i2c_smbus_data data;
    if (i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
                         I2C_SMBUS_BYTE_DATA,&data))
      return -1;
    else
      return 0x0FF & data.byte;
  }
#endif /*USE_SMBUS_API*/

const char *byte_to_binary(int x)
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


static inline void versaclock_check_los(int file_descriptor)
{

  uint8_t state_word;
  state_word = i2c_read_register(file_descriptor,VC5_SRC_STATUS_REGISTER);

  // filter so that only the relevant bits hold info
  state_word = (state_word & (VC5_LOS_ANY));

  printf("versaclock_check_los: Loss link state : \r\n");
  uint16_t res = 0x0;
  if (state_word & VC5_LOS_ALT_IN)
  {
    res = 1;
  }
  else
  {
    res = 0;
  }
  printf("\t Alternative input : %u\r\n",res);
  if (state_word & VC5_LOS_PRI_IN)
  {
    res = 1;
  }
  else
  {
    res = 0;
  }
  printf("\t Primary input : %u\r\n",res);
  if (state_word & VC5_LOS_CLKIN)
  {
    res = 1;
  }
  else
  {
    res = 0;
  }
  printf("\t Input clock : %u\r\n",res);
  if (state_word & VC5_LOS_LOCK)
  {
    res = 1;
  }
  else
  {
    res = 0;
  }
  printf("\t Signal lock : %u\r\n",res);
  if (state_word & VC5_LOS_BOTH)
  {
    res = 1;
  }
  else
  {
    res = 0;
  }
  printf("\t LOS both : %u\r\n",res);

}

static inline void versaclock_check_status(int file_descriptor)
{

  uint8_t state_word;
  state_word = i2c_read_register(file_descriptor,VC5_STATUS_REGISTER_GLOBAL);

  // filter so that only the relevant bits hold info
  state_word = (state_word & (VC5_STATUS_MASK));

  printf("versaclock_check_status: Checking status bits : \r\n");
  uint16_t res = 0x0;
  if (state_word & VC5_STATUS_INIT_DONE)
  {
    res = 1;
  }
  else
  {
    res = 0;
  }
  printf("\t Init done : %u\r\n",res);
  if (state_word & VC5_STATUS_INIT_ACTIVE)
  {
    res = 1;
  }
  else
  {
    res = 0;
  }
  printf("\t Init active : %u\r\n",res);
  if (state_word & VC5_STATUS_MARGIN_RD_FAIL)
  {
    res = 1;
  }
  else
  {
    res = 0;
  }
  printf("\t Margin read fail : %u\r\n",res);

}


static inline void dump_versaclock(int file)
{
  uint8_t i = 0;
  int rawdata;
  uint8_t data;
  printf("Sumping the register content of the system:\n");
  for (i = 0x00; i <= 0x9F; i++)
  {
    data = i2c_read_register(file, i);
    if (data < 0)
    {
      printf("ERROR: Failed on reg 0x%02X\n",i);
    }
    printf("Reg 0x%02X : [0x%02X] [%s]\n",i,data,byte_to_binary(data));
  }
}

static inline void reset_versaclock(int file)
{
  i2c_write_register_mask(file,reset_reg,0x0,reset_mask);
  i2c_write_register_mask(file,reset_reg,reset_mask,reset_mask);
  (void)usleep(VC5_TIME_CHECK_PLL_CONFIG_US);
}

static inline int32_t program_versaclock(int file)
{
  versaclock_register reg;
  int status;
  // first check that the address is the same as currently
  if (((chip_addr >> 1) & 0x1) ^ (reg_store[0].val & 0x1))
  {
    printf("Noting an address change. After this, will need to reconnect to the I2C device\n");
    i2c_write_register_mask(file,0x0,reg_store[0].val,0x1);
    chip_addr = (chip_addr & ~(0x2)) | ((reg_store[0x0].val & 0x1) << 0x1);
    printf("\r\n\r\nWARNING : Setting updated address to  0x%02X.\r\n\r\n",(unsigned int)chip_addr);
    printf("Closing the connection to the I2C device:\n");
    close(file);
    printf("Reopening the connection to the I2C device:\n");
    i2c_dev_number = (int)chip_addr;
    fd = open_i2c_device(i2c_bus_number,i2c_dev_number);
  }

  if (fd < 0)
  {
    printf("Failed to open file descriptor to the chip. Bailing out.\n");
    return -1;
  }
  printf("Programming the remainder of the registers:\n");
  // and now program each register as needed
  for (unsigned int i = 1; i < NUM_REGS_MAX; i++)
  {
    reg = reg_store[i];
    i2c_write_register(fd,reg.addr,reg.val);
    // read back and check
    uint8_t readback = i2c_read_register(fd,reg.addr);
    if (reg.val != readback)
    {
      printf("Readback discrepancy in reg 0x%02X : write [0x%02X] read [0x%02X]\n",i,reg.val,readback);
    }
  }

  (void)usleep(VC5_DELAY_AFTER_PLL_CONFIG_US);

  (void)reset_versaclock(fd);


  return 0;

}

int main(int argc, char **argv)
{
  //uint8_t dev =
  //int i2c_bus_number = 6;
  i2c_dev_number = chip_addr;
  //uint8_t addr = (uint8_t )chip_addr & 0xFF;
  //uint8_t reg = 0x0;

  char filename[13] ;
  size_t size = 11;

  if (argc > 1)
  {
    i2c_bus_number = strtoul(argv[1], NULL, 0);
  }
  if (argc > 2)
  {
    i2c_dev_number = strtoul(argv[2], NULL, 0);
  }

  if (i2c_bus_number > 9)
  {
    size = 12;
  }

  snprintf ( filename , size , "/dev/i2c-%i", i2c_bus_number) ;

  printf("Opening I2C device:\n BUS  : %i\n DEV  : %i\n FILE : %s\n\n",i2c_bus_number,i2c_dev_number,filename);
  fd = open_i2c_device ( i2c_bus_number , i2c_dev_number ) ;
  if (fd < 0)
  {
    printf("Failed to open device. Stopping.\n");
    return 0;
  }
  printf("Dumping registers:\n");
  dump_versaclock(fd);

  printf("Sending a reset...\n");
  reset_versaclock(fd);

  printf("Reprogramming VC5:\n");
  int ret = program_versaclock(fd);
  if (ret !=0)
  {
    printf("Failed to program CIB PLL\n");
  }
  printf("Dumping registers again:\n");
  dump_versaclock(fd);

  printf("Dumping LOS register:\n");
  versaclock_check_los(fd);

  printf("Dumping STATUS register:\n");
  versaclock_check_status(fd);

  return 0;
}
