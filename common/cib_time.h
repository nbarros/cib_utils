/*
 * cib_time_utils.h
 *
 *  Created on: Oct 23, 2024
 *      Author: Nuno Barros
 */

#ifndef COMMON_CIB_TIME_H_
#define COMMON_CIB_TIME_H_

#include <cstdint>
#include <string>
#include <ctime>
#include <memory>
#include <cib_data_fmt.h>

namespace cib
{
  namespace util
  {
    /**
     * @class cib_time
     * @brief Handles 64-bit timestamp operations from physical memory
     * 
     * Manages a 64-bit timestamp accessed at a physical memory address.
     * The timestamp consists of 16 ns clock periods (62.5 MHz clock).
     */
    class cib_time
    {
    public:
      /**
       * @brief Get the singleton instance of cib_time
       * @return Reference to the singleton instance
       */
      static cib_time& get();

      /**
       * @brief Initialize the singleton with the timestamp register address
       * @param timestamp_addr Physical memory address of the 64-bit timestamp register
       * @throws std::runtime_error if already initialized
       */
      static void initialize(uintptr_t timestamp_addr);

      /**
       * @brief Check if the singleton has been initialized
       * @return true if initialized, false otherwise
       */
      static bool isInitialized();

      /**
       * @brief Destructor
       */
      virtual ~cib_time();

      // Delete copy and move semantics
      cib_time(const cib_time&) = delete;
      cib_time(cib_time&&) = delete;
      cib_time& operator=(const cib_time&) = delete;
      cib_time& operator=(cib_time&&) = delete;

      /**
       * @brief Read the current 64-bit timestamp from physical memory
       * @return Raw timestamp in 16 ns clock periods
       */
      uint64_t get_timestamp();

      /**
       * @brief Get seconds since epoch from raw timestamp
       * @param raw_timestamp Timestamp in clock periods (16 ns ticks)
       * @return Seconds since epoch
       */
      int64_t get_seconds_since_epoch(uint64_t raw_timestamp);

      /**
       * @brief Get milliseconds since epoch from raw timestamp
       * @param raw_timestamp Timestamp in clock periods (16 ns ticks)
       * @return Milliseconds since epoch
       */
      int64_t get_milliseconds_since_epoch(uint64_t raw_timestamp);

      /**
       * @brief Format a timestamp to a human-readable string
       * @param raw_timestamp Timestamp in clock periods (16 ns ticks)
       * @param format_string strftime format string (default: "%a, %d %b %Y %H:%M:%S +0000")
       * @return Formatted timestamp string
       */
      std::string format_timestamp(uint64_t raw_timestamp, 
                                   const std::string& format_string = "%a, %d %b %Y %H:%M:%S +0000");

      /**
       * @brief Calculate timestamp from a formatted date string
       * @param ts Timestamp string in format specified by format_string
       * @param format_string strftime format string (default: "%Y-%m-%d %H:%M:%S")
       * @return Timestamp in clock periods (16 ns ticks)
       */
      uint64_t calc_timestamp(const std::string& ts, 
                             const std::string& format_string = "%Y-%m-%d %H:%M:%S");

      /**
       * @brief Convert bit pattern to floating point value
       * @param bits Bit pattern to convert
       * @param is_double_precision If true, interpret as double precision (64-bit), else single (32-bit)
       * @return Floating point value
       */
      static double convert_bits_to_float(uint64_t bits, bool is_double_precision = false);

      /**
       * @brief Get the clock frequency in Hz
       * @return Clock frequency (62,500,000 Hz for 16 ns periods)
       */
      static constexpr uint32_t get_clock_frequency_hz() { return 62500000; }

      /**
       * @brief Get the clock period in nanoseconds
       * @return Clock period (16 ns)
       */
      static constexpr uint32_t get_clock_period_ns() { return 16; }

      /**
       * @brief Convert CIB timestamp to OPC-UA DateTime format
       * @param raw_timestamp Timestamp in clock periods (16 ns ticks)
       * @return OPC-UA DateTime value (100 ns intervals since Windows epoch 1601-01-01)
       */
      static uint64_t to_ua_datetime(uint64_t raw_timestamp);

      /**
       * @brief Convert OPC-UA DateTime to CIB timestamp format
       * @param ua_datetime OPC-UA DateTime value (100 ns intervals since Windows epoch)
       * @return Timestamp in clock periods (16 ns ticks)
       */
      static uint64_t from_ua_datetime(uint64_t ua_datetime);

      /**
       * @brief Get seconds between Unix epoch (1970) and Windows epoch (1601)
       * @return Epoch offset in seconds (11644473600)
       */
      static constexpr int64_t get_epoch_offset_seconds() { return 11644473600LL; }

    private:
      /**
       * @brief Private constructor for singleton
       * @param timestamp_addr Physical memory address of the 64-bit timestamp register
       */
      explicit cib_time(uintptr_t timestamp_addr);

      uintptr_t m_timestamp_addr;
      std::unique_ptr<volatile uint32_t*> m_timestamp_ptr;  // Points to base of timestamp registers
      pdts_tstamp_t m_timestamp_struct;

      static cib_time* s_instance;
    };

  } // namespace util
} // namespace cib

#endif /* COMMON_CIB_TIME_H_ */