/*
 * test_struct_management.cpp
 *
 *  Created on: Jun 26, 2024
 *      Author: Nuno Barros
 */

#include <iostream>
#include <cstdio>
#include <cib_data_fmt.h>
#include <string>
#include <cstdint>
#include <cstring>

int main(int argc, char**argv)
{
  cib::daq::iols_tcp_packet_t packet;
  cib::daq::iols_trigger_t trigger;
  uint8_t m_buffer[32];
  bool valid = false;

  const int pm1 = 250505;
  const int pm2 = -539567;
  const int pm3 = 27222;

  for (size_t i = 0; i < 150; ++i)
  {
    valid = false;
    trigger.timestamp = static_cast<uint64_t>(i);
    trigger.set_pos_m1(pm1+i);
    //printf("Setting M2 to %X -> %X\n",pm2-i,cib::util::cast_from_signed((pm2-i),trigger.bitmask_m2));
    trigger.set_pos_m2(pm2-i);
//    printf("M2 content : %X %X %X \n",trigger.pos_m2_msb, trigger.pos_m2_lsb, trigger.get_pos_m2());

    trigger.set_pos_m3(pm3+i);

    packet.header.format_version = 0x1;
    packet.header.sequence_id = i;
    packet.header.packet_size = sizeof(cib::daq::iols_trigger_t) & 0xFFFF;
    std::memcpy(&m_buffer,&trigger,sizeof(trigger));
    std::memcpy(&(packet.word),&m_buffer,sizeof(cib::daq::iols_trigger_t));

    // now print

    if ((packet.word.timestamp == i) &&
        (packet.word.get_pos_m1() == (pm1+i)) &&
        (packet.word.get_pos_m2() == (pm2-i)) &&
        (packet.word.get_pos_m3() == (pm3+i)) &&
        (packet.header.sequence_id == i) &&
        (packet.header.format_version == 0x1) &&
        (packet.header.packet_size == sizeof(cib::daq::iols_trigger_t)))
    {
      valid = true;
    }

    if (valid)
    {
      printf("Word %u --> OK\n",i);
    }
    else
    {
      printf("Word %u --> FAILED\n",i);
      printf("Word %u --> %" PRIu64 " (%i) %i (%i) %i (%i) %i (%i) \n",i,
             packet.word.timestamp,
             i,
             packet.word.get_pos_m1(),(pm1+i),
             packet.word.get_pos_m2(),(pm2-i),
             packet.word.get_pos_m3(),(pm3+i)
             );
      break;
    }
  }
  return 0;
}


