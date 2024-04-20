/*
 * test_struct_msb.cpp
 *
 *  Created on: Apr 20, 2024
 *      Author: Nuno Barros
 */

#include <cstdio>
#include <cstdint>

typedef struct test_t
{
  uint8_t bit0 : 1;
  uint8_t bitscentral : 6;
  uint8_t bit7 : 1;
  test_t() {*reinterpret_cast<uint8_t*>(this) = 0xFF;}
  uint8_t get_u8() {return *reinterpret_cast<uint8_t*>(this);}
} test_t;


int main(int argc, char** argv)
{
  test_t t;
  uint8_t x = 0xAA;
  t = *reinterpret_cast<test_t*>(&x);

  printf(" Raw %X bit0 %hu bit 7 %hu\n",x,static_cast<uint16_t>(t.bit0),static_cast<uint16_t>(t.bit7));


  // now test constructor

  test_t t2;
  printf(" Raw %X bit0 %hu bit 7 %hu\n",t2.get_u8(),static_cast<uint16_t>(t2.bit0),static_cast<uint16_t>(t2.bit7));


  return 0;
}
