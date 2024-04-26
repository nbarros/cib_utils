/*
 * test_i2c.cpp
 *
 *  Created on: Apr 26, 2024
 *      Author: Nuno Barros
 */


#include <cstdint>
#include <string>
#include <cstdio>
#include <memory>

extern "C"
{
#include <fcntl.h>
#include <sys/ioctl.h>
#include <inttypes.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

}

int main()
{
  int fd = open ( "/dev/i2c-7", O_RDWR ) ;

  // select the device
  if ( ioctl ( fd , I2C_SLAVE, 0xd ) < 0 ) {
    printf ( "failed to find device %i on i2c bus %i\n" , i2c_device_number , i2c_bus_number ) ;
  }

}


