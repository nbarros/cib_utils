/*
 * ADN2814.cpp
 *
 *  Created on: Apr 20, 2024
 *      Author: Nuno Barros
 */

#include "../i2c/ADN2814.h"

namespace cib
{
  namespace i2c
  {

    ADN2814::ADN2814 ()
        {

        }

    ADN2814::~ADN2814 ()
    {
    }

    int ADN2814::get_frequency(uint32_t &rate)
    {
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
      // we cannot do this measurement, since we do nto have a reference clock
      SPDLOG_LOGGER_ERROR(m_log,"Can't measure fine frequency since we do not have a reference clock.");
      return 0x0;

      uint32_t freq;
      uint8_t tmp;
      rate = 0x0;
      int ret = read_register(0x2,tmp);
      if (ret != CIB_I2C_OK)
      {
        SPDLOG_LOGGER_ERROR(m_log,"Error reading register 0x2");
        return ret;
      }
      freq = tmp;
      ret = read_register(0x1,tmp);
      if (ret != CIB_I2C_OK)
      {
        SPDLOG_LOGGER_ERROR(m_log,"Error reading register 0x1");
        return ret;
      }
      // shift the existing value
      freq |= (freq << 8) | tmp;
      ret = read_register(0x0,tmp);
      if (ret != CIB_I2C_OK)
      {
        SPDLOG_LOGGER_ERROR(m_log,"Error reading register 0x0");
        return ret;
      }
      // shift the existing value and append the new
      freq = (freq << 8) | tmp;
      rate = freq;
      SPDLOG_LOGGER_DEBUG(m_log,"Computed fine frequency {:0.4f}",rate);

      return ret;
    }

    int ADN2814::get_data_rate(uint16_t &rate)
    {
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
      // the coarse read can only be made when LOL is deasserted

      adn2814_misc_reg reg;
      int ret = read_register(0x4,*reinterpret_cast<uint8_t*>(&reg));
      if (ret != CIB_I2C_OK)
      {
        // failed to check LOL
        // can't measure. print error and return
        SPDLOG_LOGGER_ERROR(m_log,"Failed to measure register status. Returned {:X}",ret);
        return ret;
      }

      if (reg.lol_status)
      {
        SPDLOG_LOGGER_ERROR(m_log,"CDR chip in LOL state.");
        return CIB_I2C_ErrorDeviceInvalidState;
      }

      // we're good to make the measurement
      // this is the only measurement we can do since we do not have a reference
      // clock

      rate = 0x0;

      uint8_t tmp;
      ret = read_register(0x3,tmp);
      if (ret != CIB_I2C_OK)
      {
        SPDLOG_LOGGER_ERROR(m_log,"Error reading register");
        return ret;
      }
      rate = tmp;
      ret = read_register(0x4,tmp);
      if (ret != CIB_I2C_OK)
      {
        SPDLOG_LOGGER_ERROR(m_log,"Error reading register");
        return ret;
      }
      // shift the existing value
      rate = (rate << 1) | (tmp & 0x1);
      printf("Rate output : %hu\n",rate);
      return ret;
    }

    int ADN2814::get_los_status(uint16_t &status)
    {
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
      static adn2814_misc_reg reg;
      int ret = read_register(0x4,*reinterpret_cast<uint8_t*>(&reg));
      status = static_cast<uint16_t>(reg.los_status);
      return ret;
    }
    int ADN2814::get_lol_status(uint16_t &status)
    {
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
      static adn2814_misc_reg reg;
      int ret = read_register(0x4,*reinterpret_cast<uint8_t*>(&reg));
      status = static_cast<uint16_t>(reg.lol_status);
      return ret;
    }

    int ADN2814::get_static_lol(uint16_t &value)
    {
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
      static adn2814_misc_reg reg;
      int ret = read_register(0x4,*reinterpret_cast<uint8_t*>(&reg));
      value =  static_cast<uint16_t>(reg.lol_static);
      return ret;
    }
    int ADN2814::get_drate_meas_complete(uint16_t &value)
    {
      if (!m_is_open)
      {
        return CIB_I2C_ErrorDeviceNotOpen;
      }
      // does not make sense since we do not have a reference clock
      static adn2814_misc_reg reg;
      int ret = read_register(0x4,*reinterpret_cast<uint8_t*>(&reg));
      value =  static_cast<uint16_t>(reg.dr_meas_done);
      return ret;
    }
    // now the writting calls
    void ADN2814::reset()
    {
      if (!m_is_open)
      {
        return;
      }
      // does not make sense since we do not have a reference clock
      static adn2814_ctrlb_reg reg;
      reg.reset_system = 1;
      int ret = write_register(0x9,reg.get_u8(),0x20);
      reg.reset_system = 0;
      ret = write_register(0x9,reg.get_u8(),0x20);
    }
    void ADN2814::reset_misc_2()
    {
      if (!m_is_open)
      {
        return;
      }
      // does not make sense since we do not have a reference clock
      static adn2814_ctrlb_reg reg;
      reg.reset_misc_2 = 1;
      int ret = write_register(0x9,reg.get_u8(),0x8);
      reg.reset_misc_2 = 0;
      ret = write_register(0x9,reg.get_u8(),0x8);
    }
    void ADN2814::reset_static_lol()
    {
      if (!m_is_open)
      {
        return;
      }
      // does not make sense since we do not have a reference clock
      static adn2814_ctrlb_reg reg;
      reg.reset_misc_4 = 1;
      int ret = write_register(0x9,reg.get_u8(),0x40);
      reg.reset_misc_4 = 0;
      ret = write_register(0x9,reg.get_u8(),0x40);
    }
    void ADN2814::set_lol_operation(LOLOP st)
    {
      if (!m_is_open)
      {
        return;
      }
      // does not make sense since we do not have a reference clock
      static adn2814_ctrlb_reg reg;
      reg.config_lol = static_cast<uint8_t>(st);
      int ret = write_register(0x9,reg.get_u8(),0x80);
      if (ret != CIB_I2C_OK)
      {
        SPDLOG_LOGGER_ERROR(m_log,"Failed to set LOL operation to {0}",static_cast<uint32_t>(st));
      }
    }
    void ADN2814::set_output_boost(bool boost)
    {
      if (!m_is_open)
      {
        return;
      }
      // does not make sense since we do not have a reference clock
      static adn2814_ctrlc_reg reg;
      reg.output_boost = (boost)?1:0;
      int ret = write_register(0x11,reg.get_u8(),0x1);
      if (ret != CIB_I2C_OK)
      {
        SPDLOG_LOGGER_ERROR(m_log,"Failed to set LOL operation to {0}",static_cast<uint32_t>(boost));
      }
    }
    void ADN2814::set_squelch_mode(SQUELCH mode)
    {
      if (!m_is_open)
      {
        return;
      }
      // does not make sense since we do not have a reference clock
      static adn2814_ctrlc_reg reg;
      reg.squelch_mode = static_cast<uint8_t>(mode);
      int ret = write_register(0x11,reg.get_u8(),0x2);
      if (ret != CIB_I2C_OK)
      {
        SPDLOG_LOGGER_ERROR(m_log,"Failed to set SQUELCH to {0}",static_cast<uint32_t>(mode));
      }
    }
    void ADN2814::set_config_los(LOSPOL mode)
    {
      if (!m_is_open)
      {
        return;
      }
      // does not make sense since we do not have a reference clock
      static adn2814_ctrlc_reg reg;
      reg.config_los = static_cast<uint8_t>(mode);
      int ret = write_register(0x11,reg.get_u8(),0x4);
      if (ret != CIB_I2C_OK)
      {
        SPDLOG_LOGGER_ERROR(m_log,"Failed to set LOS config to {0}",static_cast<uint32_t>(mode));
      }
    }




  } /* namespace i2c */
} /* namespace cib */
