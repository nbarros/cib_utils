/*
 * test_namespace.cpp
 *
 *  Created on: Jul 4, 2024
 *      Author: Nuno Barros
 */

#include <iostream>
#include <cstdio>

namespace dunedaq {
  #include <cib_data_fmt.h>
}

int main(int argc, char** argv)
{
  //cib::daq::iols_trigger_t t;
  dunedaq::cib::daq::iols_trigger_t t2;

  printf("Size  : %lu %lu \n",sizeof(t2),sizeof(t2));
  return 0;
}
