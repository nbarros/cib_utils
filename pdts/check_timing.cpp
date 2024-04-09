#include <cstdio>
#include <cstdint>
#include <cinttypes>

// not used
#define GPIO1_MEM_LOW   0x00A0010000
#define GPIO1_MEM_HIGH  0x00A001FFFF

// pdts debug (both registers look the same)
#define GPIO2_MEM_LOW   0x00A0020000
#define GPIO2_MEM_HIGH  0x00A002FFFF

// holds the timestamp
#define GPIO3_MEM_LOW   0x00A0030000
#define GPIO3_MEM_HIGH  0x00A003FFFF

#define CH0_OFFSET  0x0
#define CH1_OFFSET  0x8


typedef struct ts_t {
  uint32_t low;
  uint32_t high;
  uint64_t get_timestamp() {return *(reinterpret_cast<uint64_t*>(this));}
} ts_t;

// struct of GPIO2
typedef struct pdts_mon_t {
  uint32_t stat : 4;
  uint32_t ctrl_a : 7;
  uint32_t ctrl_d : 8;
  uint32_t cdr_los: 1;
  uint32_t sync : 8;
  uint32_t sync_stb :1;
  uint32_t rdy : 1;
  uint32_t sfp_tx_fault : 1;
  uint32_t sfp_los : 1;
  uint32_t get_u32() {return *(reinterpret_cast<uint32_t*>(this));}

} pdts_mon_t;

int main(int argc, char const *argv[])
{
  /* code */

  ts_t t;
  t.low = 0xFFFF;
  t.high= static_cast<uint32_t>(0xAAAAAAAAAA);

  printf("timestamp : 0x%" PRIX64 "\n",t.get_timestamp());
  pdts_mon_t mon;
  *reinterpret_cast<uint32_t*>(&mon) = static_cast<uint32_t>(0xAAAAAAAAAA);
  printf("Size %lu value : 0x%" PRIX32 "\n",sizeof(mon),mon.get_u32());


  // first map the memory
  // we do need that

  return 0;
}
