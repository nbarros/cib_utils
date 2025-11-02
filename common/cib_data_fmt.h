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
#include <cstddef>
#include <string>

using std::size_t;

#ifdef CIB_DUNEDAQ
namespace dunedaq {
#endif

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
      mapped_mem_t gpio_triggers;
    } cib_mem_t;

    // the idea of this structure is to have a convenient way to control the lbls module
    // the unused bits are marked as padding
    typedef struct lbls_ctl_t
    {
      uint32_t width : 8;
      uint32_t padding : 23;
      uint32_t enable : 1;
    } lbls_ctl_t;

    typedef struct motor_t
    {
      uint16_t index;
      int32_t pos_i;
      uint32_t dir;
    } motor_t;

    // data format for the LBLS data
    // it consists of one 64 bit timestamp followed by 
    // 48 8-bit amplitude over threshold samples
    typedef struct lbls_data_t
    {
      const static size_t num_samples = 48;
      uint8_t samples[num_samples];
      uint32_t timestamp;
    } lbls_data_t;

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
        static const uint32_t bitmask_m3 = 0x1FFFF;
        static const uint32_t mask_m2_lsb = 0xFFFE0000;
        static const uint32_t bitmask_m2_lsb = 0xEFFF;

        static const uint32_t mask_m2_msb = 0x7F;
        static const uint32_t bitmask_m2_msb = 0x3F8000;
        static const uint32_t bitmask_m2 = 0x3FFFFF;
        static const uint32_t mask_m1 = 0x1FFFFF80;
        static const uint32_t bitmask_m1 = 0x3FFFFF;

        static size_t const size_bytes = 16;

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
        uint16_t get_version() {return ((format_version >> 4 ) & 0xF);}
        void set_version(uint16_t v) {format_version = ((v & 0xF) << 4) | (~v & 0xF);}
      } tcp_header_t;

      typedef union tcp_header
      {
        tcp_header_t word;
        uint32_t     value;
      } tcp_header;


      // -- this should be the base structure to data taking and shipment
      typedef struct iols_tcp_packet_t
      {
        iols_trigger_t word;
        tcp_header_t header;
      } iols_tcp_packet_t;

      typedef struct iols_feedback_msg_t
      {
        std::string sev;
        std::string msg;
      } iols_feedback_msg_t;


    } // namespace daq

  }
#ifdef CIB_DUNEDAQ
}
#endif


#endif /* COMMON_CIB_DATA_FMT_H_ */
