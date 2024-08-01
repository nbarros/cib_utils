/*
 * cib_data_utils.cpp
 *
 *  Created on: Jul 5, 2024
 *      Author: Nuno Barros
 */

#include <cib_data_fmt.h>
#include <mem_utils.h>

int32_t get_m1(cib::daq::iols_trigger_t &t)
{
  return cib::util::cast_to_signed(t.pos_m1,t.bitmask_m1);
}

int32_t get_m2(cib::daq::iols_trigger_t &t)
{
  uint32_t m2_lsb = t.pos_m2_lsb;
  uint32_t m2_msb = t.pos_m2_msb;
  uint32_t m2 = (m2_msb << 15) | t.pos_m2_lsb;
  // the bitmask is the same
  return cib::util::cast_to_signed(m2,t.bitmask_m2);
}

int32_t get_m3(cib::daq::iols_trigger_t &t)
{
  return cib::util::cast_to_signed(t.pos_m3,t.bitmask_m3);
}


void set_pos_m1(cib::daq::iols_trigger_t &t,int32_t v)
{
  t.pos_m1 = cib::util::cast_from_signed(v,t.bitmask_m1);
}
void set_pos_m3(cib::daq::iols_trigger_t &t,int32_t v)
{
  t.pos_m3 = cib::util::cast_from_signed(v,t.bitmask_m3);
}
void set_pos_m2(cib::daq::iols_trigger_t &t,int32_t v)
{
  uint32_t tmp = cib::util::cast_from_signed(v,t.bitmask_m2);
  t.pos_m2_lsb = (tmp & t.bitmask_m2_lsb);
  t.pos_m2_msb = ((tmp & t.bitmask_m2_msb) >> 15);
}
