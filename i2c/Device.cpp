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
    {

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
      printf("Received bus %d\n",bus);
      m_device = "/dev/i2c-" + std::to_string (bus);
      printf("Device::set_bus : bus set to [%s]\n",m_device.c_str());
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
      if ((m_bus_num < 0) || (m_dev_addr < 0))
      {
        return CIB_I2C_ErrorDeviceConfig;
      }
      // if the device is already open, do nothing
      if (m_is_open)
      {
        return CIB_I2C_OK;
      }
      // open the device
      printf("Opening device [%s]\n",m_device.c_str());
      m_fd = open(m_device.c_str(),O_RDWR);
      if (m_fd < 0)
      {
        printf("Device::open_device : Failed to open device bus: %d : %s\n",errno,std::strerror(errno));
        return CIB_I2C_ErrorOpenDevice;
      }
      int ret = select_device();
      if (ret != CIB_I2C_OK)
      {
        close_device();
        return ret;
      }
        // find the specified device
      if (ioctl ( m_fd , I2C_SLAVE, m_dev_addr ) < 0 )
      {
        printf("Device::open_device :Failed to open device at address: %d : %s\n",errno,std::strerror(errno));
        close_device();
        return CIB_I2C_ErrorOpenDevice;
      }
      m_is_open = true;
      // populate the functionality array
      if (ioctl(m_fd, I2C_FUNCS, &m_dev_funcs) < 0)
      {
        printf("WARN: Failed to query device for functionality: %d : %s\n",errno,std::strerror(errno));
      }
      else
      {
        printf("Functionlity of the device: [%" PRIx64 "]\n",m_dev_funcs);
      }

      return CIB_I2C_OK;
    }
    int Device::close_device()
    {
      if (m_is_open)
      {
        if (close(m_fd) < 0)
        {
          printf("WARN: Failed to close file descriptor: %d : %s\n",errno,std::strerror(errno));
        }
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
      if ( wr_bytes != 1 )
      {
        printf ( "error: i2c write returned %i instead of 1\n" , wr_bytes ) ;
      }
      uint8_t buf[3];
      //int rd_bytes = read ( m_fd, buf, 1 ) ;
      int rd_bytes = i2c_smbus_read_byte_data(m_fd,addr);
      if ( rd_bytes == -1 )
      {
        printf ( "--> error: i2c read returned %i instead of 1\n" , rd_bytes ) ;
      }
      else
      {
	printf("Interim result : %d\n",rd_bytes);
      }

      data = rd_bytes & 0xFF;
      printf("Returning data 0x%X\n",data);
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
      int ret = select_device();
      if (ret != CIB_I2C_OK)
      {
        return ret;
      }
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
        printf("Failed to write message: %d : %s\n",errno,std::strerror(errno));
        return CIB_I2C_ErrorWriteRegister;
      }
      return CIB_I2C_OK;
    }

    int  Device::write_word_register_smbus(const uint8_t addr, const uint16_t data, const uint16_t mask)
    {
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
      int ret = select_device();
      if (ret != CIB_I2C_OK)
      {
        return ret;
      }
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
        printf("Failed to write message: %d : %s\n",errno,std::strerror(errno));
        return CIB_I2C_ErrorWriteRegister;
      }
      return CIB_I2C_OK;
    }

    int  Device::write_block_register_smbus(const uint8_t addr, const uint8_t len, const uint8_t *data)
    {
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
      int ret = select_device();
      if (ret != CIB_I2C_OK)
      {
        return ret;
      }
      // verify that the size is withing limits allowed by smbus
      if  (len > 32)
      {
        return CIB_I2C_ErrorBlockSize;
      }
      // and now just write the thing
      // zero means success
      ret = i2c_smbus_write_block_data(m_fd,addr,len,data);
      if (ret < 0)
      {
        printf("Failed to write message: %d : %s\n",errno,std::strerror(errno));
        return CIB_I2C_ErrorWriteRegister;
      }
      return CIB_I2C_OK;
    }

    int  Device::read_byte_register_smbus(const uint8_t addr, uint8_t &data)
    {
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
      int ret = select_device();
      if (ret != CIB_I2C_OK)
      {
        return ret;
      }
      ret = i2c_smbus_read_byte_data(m_fd,addr);
      if (ret < 0)
      {
        printf("Failed to read message: %d : %s\n",errno,std::strerror(errno));
        return CIB_I2C_ErrorReadRegister;
      }
      data  = ret & 0xFF;
      return CIB_I2C_OK;
    }

    int  Device::read_word_register_smbus(const uint8_t addr, uint16_t &data)
    {
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
      int ret = select_device();
      if (ret != CIB_I2C_OK)
      {
        return ret;
      }
      ret = i2c_smbus_read_word_data(m_fd,addr);
      if (ret < 0)
      {
        printf("Failed to read message: %d : %s\n",errno,std::strerror(errno));
        return CIB_I2C_ErrorReadRegister;
      }
      data  = ret & 0xFFFF;
      return CIB_I2C_OK;
    }
    int  Device::read_block_register_smbus(const uint8_t addr, uint8_t &len, uint8_t *&data)
    {
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
      int ret = select_device();
      if (ret != CIB_I2C_OK)
      {
        return ret;
      }
      if (!(m_dev_funcs & I2C_FUNC_SMBUS_READ_BLOCK_DATA))
      {
        return CIB_I2C_ErrorFuncNotSupported;
      }
      ret = i2c_smbus_read_block_data(m_fd,addr,data);
      if (ret < 0)
      {
        printf("Failed to read message: %d : %s\n",errno,std::strerror(errno));
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
      if (ioctl ( m_fd , I2C_SLAVE, m_dev_addr ) < 0 )
      {
        printf("Failed to select device [%d,0x%x]: %d : %s\n",m_bus_num,m_dev_addr,errno,std::strerror(errno));
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
      for (z = 128; z > 0; z >>= 1)
      {
        strcat(b, ((x & z) == z) ? "1" : "0");
      }

      return b;

    }

  } /* namespace i2c */
} /* namespace cib */
