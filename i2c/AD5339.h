/*
 * AD5339.h
 *
 *  Created on: Apr 21, 2024
 *      Author: Nuno Barros
 */

#ifndef I2C_AD5339_H_
#define I2C_AD5339_H_

#include <Device.h>

namespace cib
{
  namespace i2c
  {

    // pointer byte. It is zero for most stuff
    typedef struct dac_ptr_t
    { // lsb
      uint8_t dac_a   : 1;
      uint8_t dac_b   : 1;
      uint8_t zeros   : 4;
      uint8_t padding : 2;
      dac_ptr_t() {*reinterpret_cast<uint8_t*>(this) = 0x0;};
      uint8_t get_u8() {return *reinterpret_cast<uint8_t*>(this);}
    } dac_ptr_t;

    /** Power down modes- PD1, PD0
     * 0 0 : normal operation
     * 0 1 :  power down (1kOhm to GND)
     * 1 0 : power down (100 kOhm to GND)
     * 1 1 : power down (tri-state output)

     ldac_bar : 0 : command committed to DAC; 1: command only set on cache, write when next ldac_bar is 0

     clr_bar  : 0 : clear the DAC (fill with 0); 1: normal operation
     */

    // the structure arrives reversed byte aligned
//    typedef struct dac_msg_t
//    {
//      uint8_t level_lsb;
//      uint8_t level_msb : 4;
//      uint8_t ldac_bar  : 1;
//      uint8_t clr_bar : 1;
//      uint8_t pd0       : 1;
//      uint8_t pd1       : 1;
//      dac_msg_t() {*reinterpret_cast<uint16_t*>(this) = 0x0;};
//      uint16_t get_level() {return (*reinterpret_cast<uint16_t*>(this) & 0xFFF);}
//      void set_level(const uint16_t v) {level_lsb = (v & 0xFF); level_msb = ((v>>8) & 0xF);}
//      uint16_t get_u16() {return *reinterpret_cast<uint16_t*>(this);}
//    } dac_msg_t;

    typedef struct dac_msg_t
    {
      uint8_t level_msb : 4;
      uint8_t ldac_bar  : 1;
      uint8_t clr_bar : 1;
      uint8_t pd0       : 1;
      uint8_t pd1       : 1;
      uint8_t level_lsb;
      dac_msg_t() {*reinterpret_cast<uint16_t*>(this) = 0x0;};
      uint16_t get_level() {return (*reinterpret_cast<uint16_t*>(this) & 0xFFF);}
      void set_level(const uint16_t v) {level_lsb = (v & 0xFF); level_msb = ((v>>8) & 0xF);}
      uint16_t get_u16() {return *reinterpret_cast<uint16_t*>(this);}
    } dac_msg_t;

    class AD5339 final : public Device
    {
      // This DAC actually works on 16 bit mode
      // see datasheet for details
      // the inital byte does have some
    public:
      enum Channel {CH_1 = 1, CH_2 = 2, CH_ALL = 3};
      AD5339 ();
      virtual ~AD5339 ();
      AD5339 (const AD5339 &other) = delete;
      AD5339 (AD5339 &&other) = delete;
      AD5339& operator= (const AD5339 &other) = delete;
      AD5339& operator= (AD5339 &&other) = delete;

      int get_level(const Channel ch, uint16_t &value);
      int get_level(const uint16_t ch, uint16_t &value) {return get_level(static_cast<enum Channel>(ch),value);}

      int set_level(const Channel ch, const uint16_t value);
      int set_level(const uint16_t ch, const uint16_t value) {return set_level(static_cast<enum Channel>(ch),value);}

      int clear(const Channel ch);

    protected:



    };

  } /* namespace i2c */
} /* namespace cib */

#endif /* I2C_AD5339_H_ */
