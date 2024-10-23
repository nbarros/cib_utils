/*
 * time_utils.h
 *
 *  Created on: Oct 23, 2024
 *      Author: Nuno Barros
 */

#ifndef COMMON_TIME_UTILS_H_
#define COMMON_TIME_UTILS_H_

#include <cstdint>
#include <string>

namespace cib
{
  namespace util
  {
    int64_t get_seconds_since_epoch();
    //-----------------------------------------------------------------------------
    int64_t get_milliseconds_since_epoch();
    //-----------------------------------------------------------------------------
    std::string format_timestamp(uint64_t raw_timestamp, uint32_t clock_frequency_hz); // NOLINT(build/unsigned)
    //-----------------------------------------------------------------------------
    double convert_bits_to_float(uint64_t bits, bool is_double_precision); // NOLINT(build/unsigned)
    //-----------------------------------------------------------------------------
    uint64_t calc_timestamp(std::string ts, uint32_t clock_frequency_hz);
    //-----------------------------------------------------------------------------
    std::string format_timestamp(uint64_t raw_timestamp, uint32_t clock_frequency_hz); // NOLINT(build/unsigned)
    //-----------------------------------------------------------------------------
  }
}


#endif /* COMMON_TIME_UTILS_H_ */
