/*
 * time_utils.cpp
 *
 *  Created on: Oct 23, 2024
 *      Author: Nuno Barros
 */

#include <time_utils.h>
#include <cmath>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>

namespace cib
{
  namespace util
  {
    int64_t
    get_seconds_since_epoch()
    {
      // get the current time
      const auto now = std::chrono::system_clock::now();

      // transform the time into a duration since the epoch
      const auto epoch = now.time_since_epoch();

      // cast the duration into seconds
      const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(epoch);

      // return the number of seconds
      return seconds.count();
    }
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    int64_t get_milliseconds_since_epoch()
    {
      // get the current time
      const auto now = std::chrono::system_clock::now();

      // transform the time into a duration since the epoch
      const auto epoch = now.time_since_epoch();

      // cast the duration into milliseconds
      const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);

      // return the number of milliseconds
      return milliseconds.count();
    }
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    std::string format_timestamp(uint64_t raw_timestamp, uint32_t clock_frequency_hz) // NOLINT(build/unsigned)
    {
      std::time_t sec_from_epoch = raw_timestamp / clock_frequency_hz;
      printf("sec_from_epoch %ld\n",sec_from_epoch);
      struct tm* time = localtime(&sec_from_epoch); // NOLINT
      char time_buffer[32];

      std::strftime(time_buffer, sizeof time_buffer, "%a, %d %b %Y %H:%M:%S +0000", time);
      printf("DATE : %s\n",time_buffer);
      return time_buffer;
    }
    //-----------------------------------------------------------------------------
    uint64_t calc_timestamp(std::string ts, uint32_t clock_frequency_hz)
    {
      // from https://www.geeksforgeeks.org/date-and-time-parsing-in-cpp/

      tm tm_st = {};
      const std::string tm_fmt = "%Y-%m-%d %H:%M:%S";

      std::istringstream ss(ts);
      ss >> std::get_time(&tm_st, tm_fmt.c_str());
      uint64_t clock_ticks = mktime(&tm_st) * clock_frequency_hz;

      /**
       * Old, C-style code.
       * Reference:  https://www.geeksforgeeks.org/date-and-time-parsing-in-cpp/

      // we want to convert a timestamp into
      const std::string tfmt = "%Y-%m-%d %H:%M:%S";
      struct tm tm_struct;
      strptime(ts.c_str(),tfmt.c_str(),&tm_struct);
      time_t tval = mktime(&tm_struct); // sec since epoch
      // now convert this into 16 ns ticks
      uint64_t clock_ticks = tval*clock_frequency_hz;
      */
      return clock_ticks;
    }    
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    double
    convert_bits_to_float(uint64_t bits, bool is_double_precision) // NOLINT(build/unsigned)
    {
      uint32_t mantissa_shift = is_double_precision ? 52 : 23;                        // NOLINT(build/unsigned)
      uint64_t exponent_mask = is_double_precision ? 0x7FF0000000000000 : 0x7f800000; // NOLINT(build/unsigned)
      uint32_t bias = is_double_precision ? 1023 : 127;                              // NOLINT(build/unsigned)
      uint32_t sign_shift = is_double_precision ? 63 : 31;                            // NOLINT(build/unsigned)

      int32_t sign = (bits >> sign_shift) & 0x01;
      uint32_t exponent_biased = ((bits & exponent_mask) >> mantissa_shift); // NOLINT(build/unsigned)
      int32_t exponent = exponent_biased - bias;

      int32_t power = -1;
      double mantissa = 0.0;
      for (uint32_t i = 0; i < mantissa_shift; ++i) {                       // NOLINT(build/unsigned)
        uint64_t mantissa_bit = (bits >> (mantissa_shift - i - 1)) & 0x01; // NOLINT(build/unsigned)
        mantissa += mantissa_bit * pow(2.0, power);
        --power;
      }

      if (exponent_biased == 0) {
        ++exponent;
        if (mantissa == 0)
          return 0;
      } else {
        mantissa += 1.0;
      }
      return (sign ? -1 : 1) * pow(2.0, exponent) * mantissa;
    }
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
  }
}

