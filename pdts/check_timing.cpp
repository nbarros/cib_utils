#include <cstdio>
#include <cstdint>
#include <cinttypes>

#define GPIO1_MEM_LOW   0x00000A000000
#define GPIO1_MEM_HIGH  0x00000A00FFFF

#define GPIO2_MEM_LOW   0x00000A100000
#define GPIO2_MEM_HIGH  0x00000A10FFFF

#define GPIO3_MEM_LOW   0x00000A200000
#define GPIO3_MEM_HIGH  0x00000A20FFFF

#define CH0_OFFSET  0x0
#define CH1_OFFSET  0x8


typedef struct ts_t {
  uint32_t low;
  uint32_t high;
  uint64_t get_timestamp() {return *(reinterpret_cast<uint64_t*>(this));}
} ts_t;

// struct of GPIO2
typedef struct pdts_mon_t {
  uint32_t ctrl : 6;
  uint32_t reg2 : 11;
  uint32_t reg3 : 10;
  uint32_t reg4 : 5;
  uint32_t get_u32() {return *(reinterpret_cast<uint32_t*>(this));}

} pdts_mon_t;

int main(int argc, char const *argv[])
{
  /* code */

  ts_t t;
  t.low = 0xFFFF;
  t.high= 0xAAAAAAAAAA;

  printf("timestamp : 0x%" PRIX64 "\n",t.get_timestamp());
  pdts_mon_t mon;
  *reinterpret_cast<uint32_t*>(&mon) = 0xAAAAAAAAAA;
  printf("Size %lu value : 0x%" PRIX32 "\n",sizeof(mon),mon.get_u32());


  return 0;
}
