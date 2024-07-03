/*
 * Device.h
 *
 *  Created on: Apr 20, 2024
 *      Author: Nuno Barros
 */

#ifndef I2C_DEVICE_H_
#define I2C_DEVICE_H_

#include <cstdint>
#include <string>
#include <cstdio>
#include <memory>

extern "C"
{
  #include <inttypes.h>
  #include <linux/i2c.h>
  #include <linux/i2c-dev.h>
}
#include <spdlog/spdlog.h>

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
#define CIB_I2C_ErrorSelectDevice 0x200
#define CIB_I2C_ErrorBlockSize 0x400
#define CIB_I2C_ErrorFuncNotSupported 0x800
#define CIB_I2C_ErrorInvalidArgument 0x1000

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
      bool is_open() {return m_is_open;}
      int write_register(uint8_t addr, uint8_t data, uint8_t mask = 0xFF);
      int read_register(uint8_t addr, uint8_t &data);

      // this uses a slightly different set of methods with the smbus interface
      // old CTB DAC style interface
      // this uses the SMBUS protocol
      // for most part it is recommended since it is more stable
      int write_byte_register_smbus(const uint8_t addr, const uint8_t data, const uint8_t mask = 0xFF);
      int write_word_register_smbus(const uint8_t addr, const uint16_t data, const uint16_t mask = 0xFFFF);
      int write_block_register_smbus(const uint8_t addr, const uint8_t len, const uint8_t *data);
      int read_byte_register_smbus(const uint8_t addr, uint8_t &data);
      int read_word_register_smbus(const uint8_t addr, uint16_t &data);
      int read_block_register_smbus(const uint8_t addr, uint8_t &len, uint8_t *&data);

      int dump_device_memory();

    protected:
      // protetected functions.Still avilable on derived objects
      const char* byte_to_binary(uint8_t x);
      const char*  u16_to_binary(uint16_t x);
      const char*  u32_to_binary(uint32_t x);
      const char*  u64_to_binary(uint64_t x);

      int select_device();

    protected:
      bool m_is_open;
      std::string m_device;
      int m_fd;
      int m_bus_num;
      int m_dev_addr;
      uint64_t m_dev_funcs;
      std::shared_ptr<spdlog::logger> m_log;

    };

  } /* namespace i2c */
} /* namespace cib */

#endif /* I2C_DEVICE_H_ */
