#include <stdio.h>
#include <stdint.h> // uint8_t, uint16_t, uint32_t, uint64_t
#include <linux/i2c-dev.h>  // I2C_SLAVE
#include <fcntl.h> // open(), O_RDONLY
#include <unistd.h> // read(), write(), usleep()
#include <sys/ioctl.h> // ioctl()

#define I2C_BUS_NUMBER 1
#define I2C_DEVICE_NUMBER 0x1C
#define I2C_REGISTER_NUMBER 4

int open_i2c_device ( int i2c_bus_number , int i2c_device_number ) {
  char filename[12] ;
  snprintf ( filename , 11 , "/dev/i2c-%i", i2c_bus_number) ;
  int file_descriptor = open ( filename, O_RDWR ) ;
  if ( file_descriptor < 0 ) {
    printf ( "failed to open %s, open() returned %i\n" , filename , file_descriptor ) ;
  }
  if ( ioctl ( file_descriptor , I2C_SLAVE, i2c_device_number ) < 0 ) {
    printf ( "failed to find device %i on i2c bus %i\n" , i2c_device_number , i2c_bus_number ) ;
  }
  return ( file_descriptor ) ;
}

void i2c_write_register ( int file_descriptor , uint8_t register_to_write_to , uint8_t data_to_write_to_register ) {
  uint8_t message[2] ;
  message[0] = register_to_write_to ;
  message[1] = data_to_write_to_register ;
  int i = write ( file_descriptor , message , 2 ) ;
  if ( i != 2 ) {
    printf ( "error: i2c write returned %i instead of 2\n" , i ) ;
  }
}

uint8_t i2c_read_register ( int file_descriptor , uint8_t register_to_read_from ) {
  uint8_t message[1] ;
  message[0] = register_to_read_from ;
  int i = write ( file_descriptor , message , 1 ) ;
  if ( i != 1 ) {
    printf ( "error: i2c write returned %i instead of 1\n" , i ) ;
  }
  i = read ( file_descriptor , message , 1 ) ;
  if ( i != 1 ) {
    printf ( "error: i2c read returned %i instead of 1\n" , i ) ;
  }
  return ( message[0] ) ;
}

int main () {
  int file_descriptor_for_I2C_device = open_i2c_device ( I2C_BUS_NUMBER , I2C_DEVICE_NUMBER ) ;
  i2c_write_register ( file_descriptor_for_I2C_device , I2C_REGISTER_NUMBER , 0xA1 ) ;
  uint8_t i = i2c_read_register ( file_descriptor_for_I2C_device , I2C_REGISTER_NUMBER ) ;
  printf ( "register  %02hhx is %02hhx\n" , I2C_REGISTER_NUMBER, i) ;
}
