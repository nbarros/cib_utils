/*
 * Device.h
 *
 *  Created on: Apr 20, 2024
 *      Author: Nuno Barros
 */

#ifndef I2C_MISC_DEVICE_H_
#define I2C_MISC_DEVICE_H_

#include <cstdint>
#include <string>
#include <cstdio>
extern "C"
{
  #include <inttypes.h>
  #include <linux/i2c.h>
  #include <linux/i2c-dev.h>
}
#define BIT(n) (1UL << n)

namespace cib
{
  namespace i2c
  {

#define CIB_I2C_OK 0x0
#define CIB_I2C_ERROR 0x1
#define CIB_I2C_DeviceInUse 0x2
#define CIB_I2C_ErrorOpenDevice 0x4
#define CIB_I2C_ErrorDeviceConfig 0x8
#define CIB_I2C_ErrorDeviceNotOpen 0x10
#define CIB_I2C_ErrorReadRegister 0x20
#define CIB_I2C_ErrorWriteRegister 0x40
#define CIB_I2C_ErrorNotImplemented 0x80
#define CIB_I2C_ErrorDeviceInvalidState 0x100

    extern  const char* strerror(int err);

    typedef struct i2c_write_message
    {
       uint8_t addr; // address to write to
       uint8_t val;  // contents to be written
    } i2c_write_message;


    /*
     *
     */
    class Device
    {
    public:

      Device ();
      virtual ~Device ();
      Device (const Device &other) = delete;
      Device (Device &&other) = delete;
      Device& operator= (const Device &other) = delete;
      Device& operator= (Device &&other) = delete;
      int set_device(const std::string dev);
      const std::string get_device() {return m_device;}
      int set_bus(const int &bus);
      int set_dev_number(const int &dev);
      int open_device();
      int close_device();
      int write_register(uint8_t addr, uint8_t data, uint8_t mask = 0xFF);
      int read_register(uint8_t addr, uint8_t &data);

      int dump_device_memory();

    protected:
      // protetected functions.Still avilable on derived objects
      const char* byte_to_binary(uint8_t x);

    protected:
      bool m_is_open;
      std::string m_device;
      int m_fd;
      int m_bus_num;
      int m_dev_addr;
      unsigned long * m_dev_funcs;
    };

  } /* namespace i2c */
} /* namespace cib */

#endif /* I2C_MISC_DEVICE_H_ */
