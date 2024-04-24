/*
 * AD5339.cpp
 *
 *  Created on: Apr 24, 2024
 *      Author: Nuno Barros
 */

#include <AD5339.h>

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
          printf("Unknown channel %d\n",static_cast<int>(ch));
          return CIB_I2C_ErrorInvalidArgument;
      }
      uint16_t word;
      ret = read_word_register_smbus(ptr.get_u8(),word);
      if (ret != CIB_I2C_OK)
      {
        // something failed
        value = 0x0;
        return ret;
      }
      printf("Received word %hu\n",word);
      dac_msg_t msg = *reinterpret_cast<dac_msg_t*>(&word);
      printf("Recasted word %hu\n",msg.get_u16());
      printf("DAQ value : %hu\n",msg.get_level());
      value = msg.get_level();

      return CIB_I2C_OK;
    }

    int AD5339::set_level(const Channel ch, const uint16_t value)
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
          printf("Unknown channel %d\n",static_cast<int>(ch));
          return CIB_I2C_ErrorInvalidArgument;
      }
      dac_msg_t msg;
      msg.clr_bar = 1;
      msg.set_level(value);
      ret = write_word_register_smbus(ptr.get_u8(),msg.get_u16());
      if (ret != CIB_I2C_OK)
      {
        printf("Write seems to have failed.\n");
        return ret;
      }
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
          printf("Unknown channel %d\n",static_cast<int>(ch));
          return CIB_I2C_ErrorInvalidArgument;
      }
      dac_msg_t msg;
      msg.clr_bar = 0;
      ret = write_word_register_smbus(ptr.get_u8(),msg.get_u16());
      if (ret != CIB_I2C_OK)
      {
        printf("Clear seems to have failed.\n");
        return ret;
      }
      return CIB_I2C_OK;
    }

  } /* namespace i2c */
} /* namespace cib */
