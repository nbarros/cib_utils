/*
 * cib_data_utils.h
 *
 *  Created on: Aug 1, 2024
 *      Author: Nuno Barros
 */

#ifndef COMMON_CIB_DATA_UTILS_H_
#define COMMON_CIB_DATA_UTILS_H_

#include <cib_data_fmt.h>

namespace cib
{
  namespace data
  {
    int32_t get_m1(cib::daq::iols_trigger_t &t);
    int32_t get_m2(cib::daq::iols_trigger_t &t);
    int32_t get_m3(cib::daq::iols_trigger_t &t);

    void set_pos_m1(cib::daq::iols_trigger_t &t,int32_t v);
    void set_pos_m3(cib::daq::iols_trigger_t &t,int32_t v);
    void set_pos_m2(cib::daq::iols_trigger_t &t,int32_t v);
  }
}



#endif /* COMMON_CIB_DATA_UTILS_H_ */
