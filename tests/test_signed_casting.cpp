/*
 * test_signed_casting.cpp
 *
 *  Created on: May 27, 2024
 *      Author: Nuno Barros
 */

#include <mem_utils.h>
#include <random>
#include <cmath>
#undef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/spdlog.h>
#undef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

extern "C"
{
  #include <inttypes.h>
}

int main(int argc, char** argv)
{

  spdlog::set_pattern("test : [%^%L%$] %v");
  spdlog::set_level(spdlog::level::trace); // Set global log level to debug
  spdlog::trace("A trace message");
  const uint16_t msb = 21;
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 gen(rd()); // seed the generator

  std::uniform_int_distribution<> distr(-std::pow(2,msb-1), std::pow(2,msb-1)); // define the range

  spdlog::info("Generating random numbers from {0} to {1}",-std::pow(2,msb-1), std::pow(2,msb-1));
  // test random negative number that fits into the range

  uint32_t mask = cib::util::bitmask(msb,0);
  spdlog::info("Bitmask : [{0}] [0x{1:X}]",cib::util::dump_binary(mask),(1UL<<msb)-1);
  for (size_t i = 0; i < 10; i++)
  {
    int32_t r_val = distr(gen);
    spdlog::info("Generated {0} --> [{1}]",r_val,cib::util::dump_binary(r_val));

    uint32_t u_val = cib::util::cast_from_signed(r_val,mask);
    spdlog::info("converted Value : [{0}] [0x{1:X}]",cib::util::dump_binary(u_val),u_val);
    if (u_val & ~mask)
    {
      spdlog::error("Failed to convert value [{0}] remaining",cib::util::dump_binary((u_val & ~mask)));
    }
    //cast it back into a signed value
    int32_t s_val = cib::util::cast_to_signed(u_val,mask);
    spdlog::info("Re-converted Value : [{0}] [0x{1:X}] [{1}]",cib::util::dump_binary(s_val),s_val);

    if (r_val != s_val)
    {
      spdlog::warn("Conversion mismatch: real {0} conv {1}",r_val,s_val);
    }
  }


  return 0;
}


