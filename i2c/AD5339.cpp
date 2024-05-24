/*
 * AD5339.cpp
 *
 *  Created on: Apr 24, 2024
 *      Author: Nuno Barros
 */

#include <AD5339.h>

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

    AD5339::AD5339 ()
            {
            }

    AD5339::~AD5339 ()
    {
    }

    int AD5339::get_level(const Channel ch, uint16_t &value)
    {
      int ret;
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
      // doesn't this cause trouble?
      //      int ret = select_device();
      //      if (ret != CIB_I2C_OK)
      //      {
      //        return ret;
      //      }
      // remember that this actually works in two steps
      // first a pointer word and then a data word
      dac_ptr_t ptr;
      switch(ch)
      {
        case CH_1:
          ptr.dac_a = 1;
          break;
        case CH_2:
          ptr.dac_b = 1;
          break;
        case CH_ALL:
          ptr.dac_a = 1;
          ptr.dac_b = 1;
          break;
        default:
#ifdef I2C_PRINT
          SPDLOG_LOGGER_ERROR(m_log,"Unknown channel {}",static_cast<int>(ch));
#endif
          return CIB_I2C_ErrorInvalidArgument;
      }
      uint16_t word = 0x0;
#ifdef I2C_PRINT
      SPDLOG_LOGGER_DEBUG(m_log,"Getting level with [{0:x}]",ptr.get_u8());
#endif
      ret = read_word_register_smbus(ptr.get_u8(),word);
#ifdef I2C_PRINT
      SPDLOG_LOGGER_TRACE(m_log,"Got ret {0}",ret);
#endif
      if (ret != CIB_I2C_OK)
      {
        // something failed
        value = 0x0;
        return ret;
      }
#ifdef I2C_PRINT

      SPDLOG_LOGGER_TRACE(m_log,"Received word {0} [0x{0:x}]",word,word);
#endif
      dac_msg_t msg = *reinterpret_cast<dac_msg_t*>(&word);
      //SPDLOG_LOGGER_TRACE(m_log,"Recasted word {0} [0x{0:x}]",msg.get_u16(),msg.get_u16());
#ifdef I2C_PRINT
      SPDLOG_LOGGER_DEBUG(m_log,"DAC value : {0} [0x{0:x}]",msg.get_level(),msg.get_level());
#endif
      value = msg.get_level();

      return CIB_I2C_OK;
    }

    int AD5339::set_level(const Channel ch, const uint16_t value)
    {
      int ret;
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
      dac_ptr_t ptr;
      switch(ch)
      {
        case CH_1:
          ptr.dac_a = 1;
          break;
        case CH_2:
          ptr.dac_b = 1;
          break;
        case CH_ALL:
          ptr.dac_a = 1;
          ptr.dac_b = 1;
          break;
        default:
#ifdef I2C_PRINT
          SPDLOG_LOGGER_ERROR(m_log,"Unknown channel {}",static_cast<int>(ch));
#endif
          return CIB_I2C_ErrorInvalidArgument;
      }
      dac_msg_t msg;
      msg.clr_bar = 1;
      msg.set_level(value);
#ifdef I2C_PRINT
      SPDLOG_LOGGER_TRACE(m_log,"Calling write with args [0x{0:x}] [0x{1:x}] (crosscheck ch 0x{2:x} val 0x{3:x})",ptr.get_u8(),msg.get_u16(),static_cast<int>(ch),value);
#endif
      //printf("About to write 0x%X 0x%X (crosscheck : 0x%X 0x%X)\n",ptr.get_u8(),msg.get_u16(),static_cast<int>(ch),value);
      ret = write_word_register_smbus(ptr.get_u8(),msg.get_u16());
      if (ret != CIB_I2C_OK)
      {
#ifdef I2C_PRINT
        SPDLOG_LOGGER_ERROR(m_log,"Write reported failure");
#endif
        return ret;
      }
#ifdef I2C_PRINT

      SPDLOG_LOGGER_TRACE(m_log,"Write complete");
#endif
      return CIB_I2C_OK;
    }

    int AD5339::clear(const Channel ch)
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
      // remember that this actually words in two steps
      // first a pointer word and then a data word
      dac_ptr_t ptr;
      switch(ch)
      {
        case CH_1:
          ptr.dac_a = 1;
          break;
        case CH_2:
          ptr.dac_b = 1;
          break;
        case CH_ALL:
          ptr.dac_a = 1;
          ptr.dac_b = 1;
          break;
        default:
#ifdef I2C_PRINT
          SPDLOG_LOGGER_ERROR(m_log,"Unknown channel {}",static_cast<int>(ch));
#endif
          return CIB_I2C_ErrorInvalidArgument;
      }
      dac_msg_t msg;
      msg.clr_bar = 0;
      ret = write_word_register_smbus(ptr.get_u8(),msg.get_u16());
      if (ret != CIB_I2C_OK)
      {
#ifdef I2C_PRINT
        SPDLOG_LOGGER_ERROR(m_log,"Write reported failure");
#endif
        return ret;
      }
      return CIB_I2C_OK;
    }

  } /* namespace i2c */
} /* namespace cib */
