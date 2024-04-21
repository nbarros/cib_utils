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

    // this is register 0x4
    typedef struct adn2814_misc_reg
    { // LSB
      uint8_t rate_lsb : 1;      // COARSE_RD[0]
      uint8_t padding2: 1;
      uint8_t dr_meas_done : 1;  // 1: measurement complete; 0: measuring data rate
      uint8_t lol_status :1;     // 1: acquiring; 0: locked
      uint8_t lol_static : 1;    // 1: static LOL until reset; 0: waiting for next LOL
      uint8_t los_status : 1;    // 1: LOS, 0: No LOS
      uint8_t padding1 : 2;
      adn2814_misc_reg() {*reinterpret_cast<uint8_t*>(this) = 0x0;}
      uint8_t get_u8() {return *reinterpret_cast<uint8_t*>(this);}
      // MSB
    } adn2814_misc_reg;

    // register 0x8 : W
    typedef struct adn2814_ctrla_reg
    {
      // the two next bits are mutually exclusive. Either lock to reference
      // or measure the rate
      uint8_t lock_to_ref :1;     // 1: lock to reference clock; 0: lock to data
      uint8_t meas_rate : 1;      // 1: start measuring rate
      uint8_t dr_divf_ratio : 4;  // 0x0 : 1, 0x1 : 2; 0x4 : 4; 0x8 : 256; other : 2^n
      uint8_t fref_range : 2;     // 0x0: 10-20; 0x1: 20-40; 0x2: 40-80; 0x3: 80-160 MHz
      adn2814_ctrla_reg() {*reinterpret_cast<uint8_t*>(this) = 0x0;}
      uint8_t get_u8() {return *reinterpret_cast<uint8_t*>(this);}
    } adn2814_ctrla_reg;

    // register 0x9 : W
    typedef struct adn2814_ctrlb_reg
    {
      uint8_t padding2 : 3;       // must be 0
      uint8_t reset_misc_2 : 1;   // set to 1, and then back to 0 to reset MISC[2] (data rate measurement)
      uint8_t padding1 :1;        // must be 0
      uint8_t reset_system : 1;   // set to 1 and then 0 to reset everything
      uint8_t reset_misc_4 : 1;   // set to 1, and then back to 0 to reset MISC[4] (static LOL)
      uint8_t config_lol : 1;     // 1: LOl is static; 0: LOL normal operation
      adn2814_ctrlb_reg() {*reinterpret_cast<uint8_t*>(this) = 0x0;}
      uint8_t get_u8() {return *reinterpret_cast<uint8_t*>(this);}
    } adn2814_ctrlb_reg;

    // register 0x11 : W
    typedef struct adn2814_ctrlc_reg
    {
      uint8_t output_boost : 1;   // 1: boost output swing; 0: default output swing
      uint8_t squelch_mode :1;    // 1: squelch clock or data; 0: squelch clk and data
      uint8_t config_los : 1;     // 1: active high LOS; 0: active low LOS
      uint8_t padding : 5;        // set to 0
      adn2814_ctrlc_reg() {*reinterpret_cast<uint8_t*>(this) = 0x0;}
      uint8_t get_u8() {return *reinterpret_cast<uint8_t*>(this);}
    } adn2814_ctrlc_reg;

    /*
     * Register description: https://www.analog.com/media/en/technical-documentation/data-sheets/AD5339.pdf
     * 0x0[R] : Freq[7:0]
     * 0x1[R] : Freq[7:0]
     * 0x2[R] : 0 Freq[6:0]
     * 0x3[R] :
     *
     */
    class AD5339 final : public Device
    {
    public:
      AD5339 ();
      virtual ~AD5339 ();
      AD5339 (const AD5339 &other) = delete;
      AD5339 (AD5339 &&other) = delete;
      AD5339& operator= (const AD5339 &other) = delete;
      AD5339& operator= (AD5339 &&other) = delete;

      int get_frequency(uint32_t &rate);
      int get_data_rate(uint16_t &rate);
      int get_los_status(uint16_t &status);
      int get_lol_status(uint16_t &status);


      // other getters with more specific usage
      //
      int get_static_lol(uint16_t &value);
      int get_drate_meas_complete(uint16_t & value);



      void reset();
      void reset_misc_2();

      void reset_static_lol();
      void reset_misc_4() {reset_static_lol();}
      enum LOLOP {kStatic=1, kNormal=0};
      void set_lol_operation(LOLOP st);
      void set_output_boost(bool boost = false);
      enum SQUELCH {kAnd=0, kOr=1};
      void set_squelch_mode(SQUELCH mode);
      enum LOSPOL {kActiveHigh = 1, kActiveLow=0};
      void set_config_los(LOSPOL mode);



    protected:

    };

  } /* namespace i2c */
} /* namespace cib */

#endif /* I2C_AD5339_H_ */
