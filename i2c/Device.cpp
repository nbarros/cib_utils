/*
 * Device.cpp
 *
 *  Created on: Apr 20, 2024
 *      Author: Nuno Barros
 */

#include <Device.h>

#include <cerrno>
#include <cstring>
#include <cstring>

extern "C"
{
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <i2c/smbus.h>
}
#include <spdlog/sinks/stdout_color_sinks.h>

namespace cib
{
  namespace i2c
  {


    const char* strerror(int err)
    {
      static std::string msg;
      switch(err)
      {
        case CIB_I2C_OK:
          msg = "Command successful!";
          break;
        case CIB_I2C_ERROR:
          msg = "Error";
          break;
        case CIB_I2C_DeviceInUse:
          msg = "Device already in use.";
          break;
        case CIB_I2C_ErrorOpenDevice:
          msg = "Error opening device."; break;
        case CIB_I2C_ErrorDeviceConfig:
          msg = "Error configuring device."; break;
        case CIB_I2C_ErrorDeviceNotOpen:
          msg = "Device not open yet."; break;
        case CIB_I2C_ErrorReadRegister:
          msg = "Error reading register"; break;
        case CIB_I2C_ErrorWriteRegister:
          msg = "Error writing to register"; break;
        case CIB_I2C_ErrorNotImplemented:
          msg = "Functionality not implemented"; break;
        case CIB_I2C_ErrorDeviceInvalidState:
          msg = "Device is in invalid state"; break;
        case CIB_I2C_ErrorSelectDevice:
          msg = "Error selecting device"; break;
        case CIB_I2C_ErrorBlockSize:
          msg = "Error in block size"; break;
        case CIB_I2C_ErrorFuncNotSupported:
          msg = "Functionality not supported by device"; break;
        case CIB_I2C_ErrorInvalidArgument:
          msg = "Invalid argument"; break;
        default:
          msg = "Unknown error code";
          break;
      }
      return msg.c_str();
    }


    Device::Device ()
    : m_is_open(false)
    ,m_device("")
    ,m_fd(-1)
    ,m_bus_num(-1)
    ,m_dev_addr(-1)
    ,m_dev_funcs(0)
#ifdef I2C_PRINT
    ,m_log(nullptr)
#endif
    {
#ifdef I2C_PRINT
      m_log = spdlog::get("cib_i2c");
      if (m_log == nullptr)
      {
        // if it does not yet exist, create it
        m_log = spdlog::stdout_color_mt("cib_i2c");
        m_log->set_pattern("cib_i2c : [%s:%!:%#][%^%L%$] %v");
        m_log->set_level(spdlog::level::trace);
        SPDLOG_LOGGER_INFO(m_log,"Created new logger for cib_i2c");

      }
#endif
    }

    Device::~Device ()
    {
      close_device();
    }

    int  Device::set_device(const std::string dev)
    {
      if (m_is_open)
      {
        return CIB_I2C_DeviceInUse;
      }
      m_device = dev;
      return CIB_I2C_OK;
    }
    int  Device::set_bus(const int &bus)
    {
      if (m_is_open)
      {
        return CIB_I2C_DeviceInUse;
      }
#ifdef I2C_PRINT
      SPDLOG_LOGGER_TRACE(m_log,"Received bus {}",bus);
#endif
      m_device = "/dev/i2c-" + std::to_string (bus);
#ifdef I2C_PRINT
      SPDLOG_LOGGER_TRACE(m_log,"Bus set to [{0}]",m_device.c_str());
#endif
      m_bus_num = bus;
      return CIB_I2C_OK;
    }
    int  Device::set_dev_number(const int &dev)
    {
      if (m_is_open)
      {
        return CIB_I2C_DeviceInUse;
      }
      m_dev_addr = dev;
      return CIB_I2C_OK;
    }
    int  Device::open_device()
    {
      int ret;

      if ((m_bus_num < 0) || (m_dev_addr < 0))
      {
#ifdef I2C_PRINT
        SPDLOG_LOGGER_TRACE(m_log,"Device not ready to open [bus {0}, dev 0x{1:x}]",m_bus_num,m_dev_addr);
#endif
        return CIB_I2C_ErrorDeviceConfig;
      }
      // if the device is already open, do nothing
      if (m_is_open)
      {
        return CIB_I2C_OK;
      }
      // open the device
#ifdef I2C_PRINT
      SPDLOG_LOGGER_DEBUG(m_log,"Opening device [{0}]",m_device.c_str());
#endif
      m_fd = open(m_device.c_str(),O_RDWR);
      if (m_fd < 0)
      {
#ifdef I2C_PRINT
        SPDLOG_LOGGER_ERROR(m_log,"Failed to open device bus: [{0} : {1}]",errno,std::strerror(errno));
#endif
        return CIB_I2C_ErrorOpenDevice;
      }
      m_is_open = true;
      ret = select_device();
      if (ret != CIB_I2C_OK)
      {
        close_device();
        return ret;
      }
      // populate the functionality array
#ifdef I2C_PRINT
      if (ioctl(m_fd, I2C_FUNCS, &m_dev_funcs) < 0)
      {
        SPDLOG_LOGGER_WARN(m_log,"Couldn't query device functionlity [{0} : {1}]",errno,std::strerror(errno));
      }
      else
      {
        SPDLOG_LOGGER_INFO(m_log,"Device functionality : [{0}] [{1}]",m_dev_funcs,u64_to_binary(m_dev_funcs));
      }
#else
      ioctl(m_fd, I2C_FUNCS, &m_dev_funcs);
#endif
      return CIB_I2C_OK;
    }
    int Device::close_device()
    {
      if (m_is_open)
      {
#ifdef I2C_PRINT
        if (close(m_fd) < 0)
        {
          SPDLOG_LOGGER_WARN(m_log,"Failed to close file descriptor : [{0} : {1}]",errno,std::strerror(errno));
        }
#else
        close(m_fd);
#endif
        m_is_open = false;
      }
      return CIB_I2C_OK;
    }
    int  Device::write_register(uint8_t addr, uint8_t data, uint8_t mask)
    {
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
      i2c_write_message msg;
      msg.addr = addr;
      uint8_t cache;
      if (mask != 0xFFFF)
      {
        if (read_register(addr,cache))
        {
          return CIB_I2C_ErrorReadRegister;
        }
        // calculate the masked contents
        msg.val = ((cache & ~mask) | (data & mask));
      }
      // and now just write the thing
      int bytes_written = write( m_fd , reinterpret_cast<void*>(&msg), 2 ) ;
      if (bytes_written != 2)
      {
        return CIB_I2C_ErrorWriteRegister;
      }
      return CIB_I2C_OK;
    }
    int  Device::read_register(uint8_t addr, uint8_t &data)
    {
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
      int wr_bytes = write ( m_fd , &addr , 1 ) ;
#ifdef I2C_PRINT
      if ( wr_bytes != 1 )
      {
        SPDLOG_LOGGER_ERROR(m_log,"i2c write returned [{0}], instead of 1",wr_bytes);
      }
#endif
      uint8_t buf[3];
      //int rd_bytes = read ( m_fd, buf, 1 ) ;
      int rd_bytes = i2c_smbus_read_byte_data(m_fd,addr);
#ifdef I2C_PRINT
      if ( rd_bytes == -1 )
      {
        SPDLOG_LOGGER_ERROR(m_log,"i2c read returned [{0}], instead of 1",rd_bytes);
      }
      else
      {
        SPDLOG_LOGGER_DEBUG(m_log,"Readback: [{0} 0x{0:x}]",rd_bytes);
      }
#endif
      data = rd_bytes & 0xFF;
#ifdef I2C_PRINT
      SPDLOG_LOGGER_DEBUG(m_log,"Value: [{0} 0x{0:x}]",data);
#endif
      return CIB_I2C_OK;
    }

    int  Device::dump_device_memory()
    {
      return CIB_I2C_ErrorNotImplemented;
    }

    // -- smbus interface. Use this unless you need something specific
    int  Device::write_byte_register_smbus(const uint8_t addr, const uint8_t data, const uint8_t mask)
    {
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
      int ret;
//      = select_device();
//      if (ret != CIB_I2C_OK)
//      {
//        return ret;
//      }
      uint8_t msg = data;
      uint8_t cache;
      if (mask != 0xFFFF)
      {
        if (read_byte_register_smbus(addr,cache))
        {
          return CIB_I2C_ErrorReadRegister;
        }
        // calculate the masked contents
        msg = ((cache & ~mask) | (data & mask));
      }
      // and now just write the thing
      // zero means success
      ret = i2c_smbus_write_byte_data(m_fd,addr,msg);
      if (ret)
      {
#ifdef I2C_PRINT
        SPDLOG_LOGGER_ERROR(m_log,"Failed to write message [{0} : {1}]",errno,std::strerror(errno));
#endif
        return CIB_I2C_ErrorWriteRegister;
      }
      return CIB_I2C_OK;
    }

    int  Device::write_word_register_smbus(const uint8_t addr, const uint16_t data, const uint16_t mask)
    {
      int ret;

      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
//      int ret = select_device();
//      if (ret != CIB_I2C_OK)
//      {
//        return ret;
//      }
      uint16_t msg = data;
      uint16_t cache;
      if (mask != 0xFFFF)
      {
        if (read_word_register_smbus(addr,cache))
        {
          return CIB_I2C_ErrorReadRegister;
        }
        // calculate the masked contents
        msg = ((cache & ~mask) | (data & mask));
      }
      // and now just write the thing
      // zero means success
      ret = i2c_smbus_write_word_data(m_fd,addr,msg);
      if (ret)
      {
#ifdef I2C_PRINT
        SPDLOG_LOGGER_ERROR(m_log,"Failed to write message [{0} : {1}]",errno,std::strerror(errno));
#endif
        return CIB_I2C_ErrorWriteRegister;
      }
      return CIB_I2C_OK;
    }

    int  Device::write_block_register_smbus(const uint8_t addr, const uint8_t len, const uint8_t *data)
    {
      int ret;
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
//      int ret = select_device();
//      if (ret != CIB_I2C_OK)
//      {
//        return ret;
//      }
      // verify that the size is withing limits allowed by smbus
      if  (len > 32)
      {
        return CIB_I2C_ErrorBlockSize;
      }
      // and now just write the thing
      // zero means success
#ifdef I2C_PRINT
      SPDLOG_LOGGER_TRACE(m_log,"Calling write with args: [{0} {1} 0x{1:x} {2} 0x{2:x} {3} 0x{3:x}]",m_fd,addr,len,fmt::ptr(data));
#endif
      ret = i2c_smbus_write_block_data(m_fd,addr,len,data);
#ifdef I2C_PRINT
      SPDLOG_LOGGER_DEBUG(m_log,"Write call returned: [{0}]",ret);
#endif
      if (ret < 0)
      {
#ifdef I2C_PRINT
        SPDLOG_LOGGER_ERROR(m_log,"Failed to write message [{0} : {1}]",errno,std::strerror(errno));
#endif
        return CIB_I2C_ErrorWriteRegister;
      }
      return CIB_I2C_OK;
    }

    int  Device::read_byte_register_smbus(const uint8_t addr, uint8_t &data)
    {
      int ret;

      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
//      int ret = select_device();
//      if (ret != CIB_I2C_OK)
//      {
//        return ret;
//      }
      ret = i2c_smbus_read_byte_data(m_fd,addr);
      if (ret < 0)
      {
#ifdef I2C_PRINT
        SPDLOG_LOGGER_ERROR(m_log,"Failed to read message [{0} : {1}]",errno,std::strerror(errno));
#endif
        return CIB_I2C_ErrorReadRegister;
      }
      data  = ret & 0xFF;
      return CIB_I2C_OK;
    }

    int  Device::read_word_register_smbus(const uint8_t addr, uint16_t &data)
    {
      int ret;
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
//      int ret = select_device();
//      if (ret != CIB_I2C_OK)
//      {
//        return ret;
//      }
#ifdef I2C_PRINT
      SPDLOG_LOGGER_DEBUG(m_log,"Calling read word with [fd {0} : add 0x{1:x}]",m_fd,addr);
#endif
      ret = i2c_smbus_read_word_data(m_fd,addr);
      if (ret < 0)
      {
#ifdef I2C_PRINT
        SPDLOG_LOGGER_ERROR(m_log,"Failed to read message [{0} : {1}]",errno,std::strerror(errno));
#endif
        return CIB_I2C_ErrorReadRegister;
      }
#ifdef I2C_PRINT
      SPDLOG_LOGGER_DEBUG(m_log,"Received output [{0} : 0x{0:x}]",ret);
#endif
      data  = ret & 0xFFFF;
#ifdef I2C_PRINT
      SPDLOG_LOGGER_DEBUG(m_log,"Recasted output [{0} : 0x{0:x}]",data);
#endif
      return CIB_I2C_OK;
    }
    int  Device::read_block_register_smbus(const uint8_t addr, uint8_t &len, uint8_t *&data)
    {
      int ret;
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
//      int ret = select_device();
//      if (ret != CIB_I2C_OK)
//      {
//        return ret;
//      }
      if (!(m_dev_funcs & I2C_FUNC_SMBUS_READ_BLOCK_DATA))
      {
        return CIB_I2C_ErrorFuncNotSupported;
      }
      ret = i2c_smbus_read_block_data(m_fd,addr,data);
      if (ret < 0)
      {
#ifdef I2C_PRINT
        SPDLOG_LOGGER_ERROR(m_log,"Failed to read message [{0} : {1}]",errno,std::strerror(errno));
#endif
        return CIB_I2C_ErrorReadRegister;
      }
      len = ret & 0xFFFF;
      return CIB_I2C_OK;
    }


    // -- end of SMBUS interface

    int Device::select_device()
    {
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
#ifdef I2C_PRINT
      SPDLOG_LOGGER_TRACE(m_log,"Selecting device at address [{0} : {1} 0x{1:x}]",m_fd,m_dev_addr);
#endif
      if (ioctl ( m_fd , I2C_SLAVE, m_dev_addr ) < 0 )
      {
#ifdef I2C_PRINT
        SPDLOG_LOGGER_ERROR(m_log,"Failed to select device [{0} : 0x{1:x}] : {2} : {3}",m_bus_num,m_dev_addr,errno,std::strerror(errno));
#endif
        close_device();
        return CIB_I2C_ErrorSelectDevice;
      }
      return CIB_I2C_OK;
    }

    const char*  Device::byte_to_binary(uint8_t x)
    {
      static char b[9];
      b[0] = '\0';

      int z;
      for (z = (1UL<<7); z > 0; z >>= 1)
      {
        strcat(b, ((x & z) == z) ? "1" : "0");
      }

      return b;
    }
    const char*  Device::u64_to_binary(uint64_t x)
    {
      static char b[65];
      b[0] = '\0';

      uint64_t z;
      for (z = (1UL<<63); z > 0; z >>= 1)
      {
        strcat(b, ((x & z) == z) ? "1" : "0");
      }
      return b;
    }
    const char*  Device::u16_to_binary(uint16_t x)
    {
      static char b[17];
      b[0] = '\0';

      int z;
      for (z = (1UL<<15); z > 0; z >>= 1)
      {
        strcat(b, ((x & z) == z) ? "1" : "0");
      }
      return b;
    }
    const char*  Device::u32_to_binary(uint32_t x)
    {
      static char b[33];
      b[0] = '\0';

      int z;
      for (z = (1UL<<31); z > 0; z >>= 1)
      {
        strcat(b, ((x & z) == z) ? "1" : "0");
      }
      return b;
    }


  } /* namespace i2c */
} /* namespace cib */
