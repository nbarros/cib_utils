/*
 * cib_data_fmt.h
 *
 *  Created on: Jun 5, 2024
 *      Author: Nuno Barros
 */

#ifndef COMMON_CIB_DATA_FMT_H_
#define COMMON_CIB_DATA_FMT_H_
extern "C"
{
  #include <inttypes.h>
}
#include <cstdint>
#include <mem_utils.h>

namespace cib
{
  typedef struct mapped_mem_t
  {
    uintptr_t p_addr;
    uintptr_t v_addr;
  } mapped_mem_t;

  typedef struct cib_mem_t
  {
  //  mapped_mem_t config;
    mapped_mem_t gpio_pdts;
    mapped_mem_t gpio_align;
    mapped_mem_t gpio_laser;
    mapped_mem_t gpio_misc;
    mapped_mem_t gpio_mon_0;
    mapped_mem_t gpio_mon_1;
    mapped_mem_t gpio_motor_1;
    mapped_mem_t gpio_motor_2;
    mapped_mem_t gpio_motor_3;
  } cib_mem_t;

  typedef struct motor_t
  {
    uint16_t index;
    int32_t pos_i;
    uint32_t dir;
  } motor_t;

  namespace limits
  {

    typedef struct motor_limits_t
    {
      int32_t m1_min;
      int32_t m1_max;
      int32_t m2_min;
      int32_t m2_max;
      int32_t m3_min;
      int32_t m3_max;
    } motor_limits_t;

    motor_limits_t limits_p1;
    motor_limits_t limits_p2;
    motor_limits_t m_limits;
  }
  namespace daq
  {

    typedef struct iols_trigger_t
    {
      // lsb
      uint32_t pos_m3 : 17;
      uint32_t pos_m2_lsb : 15;
      uint32_t pos_m2_msb : 7;
      uint32_t pos_m1 : 22;
      uint32_t padding : 3;
      uint64_t timestamp;
      // msb
      static const uint32_t mask_m3 = 0x1FFFF;
      static const uint32_t mask_m2_1 = 0xFFFE0000;
      static const uint32_t mask_m2_2 = 0x7F;
      static const uint32_t bitmask_m2 = 0x3FFFFF;
      static const uint32_t mask_m1 = 0x1FFFFF80;
      static const uint32_t bitmask_m1 = 0x3FFFFF;

      static size_t const size_bytes = 16;


      int32_t get_pos_m1() {return cib::util::cast_to_signed(pos_m1,bitmask_m1);}
      int32_t get_pos_m3() {return cib::util::cast_to_signed(pos_m3,mask_m3);}
      int32_t get_pos_m2()
      {
        uint32_t m2_lsb = pos_m2_lsb;
        uint32_t m2_msb = pos_m2_msb;
        uint32_t m2 = (m2_msb << 15) | pos_m2_lsb;
        // the bitmask is the same
        return cib::util::cast_to_signed(m2,bitmask_m2);
      }
      //uint8_t *get_bytes() {return reinterpret_cast<uint8_t*>(&(*this));}
    } iols_trigger_t;

    ///
    /// -- TCP header
    ///

    typedef struct tcp_header_t
    {
        typedef uint16_t pkt_size_t;
        typedef uint8_t   seq_size_t;
        typedef uint8_t   ver_size_t;
        uint16_t packet_size    : 16; // Size of the data content in bytes
        uint8_t  sequence_id    : 8; // packet order...rotates every 256
        uint8_t  format_version : 8;
        static size_t const size_bytes = sizeof(uint32_t);
        static size_t const n_bits_size = 16;
        static size_t const n_bits_sequence_id = 8;
        static size_t const n_bits_version = 8;
    } tcp_header_t;

    typedef union tcp_header
    {
        tcp_header_t word;
        uint32_t     value;
    } tcp_header;


  }

}



#endif /* COMMON_CIB_DATA_FMT_H_ */
