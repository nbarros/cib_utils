#include <cstdio>
#include <cstdint>
#include <cinttypes>

#include <mem_utils.h>
#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>


///
///
///
///
///
///  NOTE: This application is no longer working
///
///
///
///
///
///


// reg 0 ised for reset
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

volatile std::atomic<bool> g_stop;

typedef struct pdts_ts_t {
  uint32_t low;
  uint32_t high;
  uint64_t get_timestamp() {return *(reinterpret_cast<uint64_t*>(this));}
} pdts_ts_t;

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

typedef struct pdts_ctl_t {
  uint32_t padding : 31;
  uint32_t reset : 1;
  uint32_t get_u32() {return *(reinterpret_cast<uint32_t*>(this));}

} pdts_ctl_t;

void test_structures()
{
  pdts_ts_t t;
  t.low = 0xFFFF;
  t.high= static_cast<uint32_t>(0xAAAAAAAAAA);
  printf("timestamp : 0x%" PRIX64 "\n",t.get_timestamp());
  pdts_mon_t mon;
  *reinterpret_cast<uint32_t*>(&mon) = static_cast<uint32_t>(0xAAAAAAAAAA);
  printf("Size %lu value : 0x%" PRIX32 "\n",sizeof(mon),mon.get_u32());

}


void sig_handler(int sig)
{
  printf("* Received signal %i *\n",sig);
  g_stop.store(true);
}

int main(int argc, char const *argv[])
{
  g_stop.store(false);

  printf("> Registering an interrupt signal\n");
  std::signal(SIGTERM,sig_handler);
  printf("> Initializing memory structures\n");

  int memfd = 0;
  printf(">> Mapping timestamp region\n");
  uintptr_t vmem_ts = cib::util::map_phys_mem(memfd,GPIO3_MEM_LOW,GPIO3_MEM_HIGH);
  printf(">>> Got 0x%" PRIXPTR "\n",vmem_ts);
  if (vmem_ts == 0x0)
  {
    return 255;
  }
  printf(">> Mapping debug registers\n");
  uintptr_t vmem_mon = cib::util::map_phys_mem(memfd,GPIO2_MEM_LOW,GPIO2_MEM_HIGH);
  printf(">>> Got 0x%" PRIXPTR "\n",vmem_mon);
  if (vmem_mon == 0x0)
  {
    cib::util::unmap_mem(cib::util::cast_to_void(vmem_ts),(GPIO3_MEM_HIGH-GPIO3_MEM_LOW));
    return 255;
  }

  printf(">> Mapping control register\n");
  uintptr_t vmem_ctl = cib::util::map_phys_mem(memfd,GPIO1_MEM_LOW,GPIO1_MEM_HIGH);
  printf(">>> Got 0x%" PRIXPTR "\n",vmem_ctl);
  if (vmem_ctl == 0x0)
  {
    cib::util::unmap_mem(cib::util::cast_to_void(vmem_ts),(GPIO3_MEM_HIGH-GPIO3_MEM_LOW));
    cib::util::unmap_mem(cib::util::cast_to_void(vmem_mon),(GPIO2_MEM_HIGH-GPIO2_MEM_LOW));
    return 255;
  }

  //
  pdts_ts_t ts;
  pdts_mon_t mon;
  pdts_ctl_t ctl;
  bool first = true;
  while (!g_stop.load())
  {
    ts.low = cib::util::reg_read((vmem_ts+CH0_OFFSET));
    ts.high = cib::util::reg_read((vmem_ts+CH1_OFFSET));

    // now grab the monitor and dump it
    *reinterpret_cast<uint32_t*>(&mon) = cib::util::reg_read((vmem_mon+CH0_OFFSET));


    printf("ts : %" PRIu64 "   stat: %" PRIX32 "\n", ts.get_timestamp(), mon.get_u32());
    printf("stat %X ctl_a %X ctl_d %X cdr_los %X sync %X sync_stb %X rdy %X sfp_tx_fault %X, sfp_los %X \n",
           mon.stat,
           mon.ctrl_a,
           mon.ctrl_d,
           mon.cdr_los,
           mon.sync,
           mon.sync_stb,
           mon.rdy,
           mon.sfp_tx_fault,
           mon.sfp_los);

    if (first)
    {
      printf("> Applying reset \n");
      *reinterpret_cast<uint32_t*>(&ctl) = cib::util::reg_read((vmem_ctl+CH0_OFFSET));
      printf(">> Before : %" PRIX32 "\n",ctl.get_u32());
      ctl.reset = 1;
      cib::util::reg_write_mask(vmem_ctl,ctl.get_u32(),(1<<31));
      std::this_thread::sleep_for(std::chrono::microseconds(2));
      *reinterpret_cast<uint32_t*>(&ctl) = cib::util::reg_read((vmem_ctl+CH0_OFFSET));
      printf(">> During : %" PRIX32 "\n",ctl.get_u32());
      ctl.reset = 0;
      cib::util::reg_write_mask(vmem_ctl,ctl.get_u32(),(1<<31));
      *reinterpret_cast<uint32_t*>(&ctl) = cib::util::reg_read((vmem_ctl+CH0_OFFSET));
      printf(">> After : %" PRIX32 "\n",ctl.get_u32());
    }
   std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // cancelation called
  // unamp memory
  cib::util::unmap_mem(cib::util::cast_to_void(vmem_ts),(GPIO3_MEM_HIGH-GPIO3_MEM_LOW));
  cib::util::unmap_mem(cib::util::cast_to_void(vmem_mon),(GPIO2_MEM_HIGH-GPIO2_MEM_LOW));

  return 0;
}
