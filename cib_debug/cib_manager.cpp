/*
 * cib_manager.cpp
 *
 *  Created on: May 9, 2024
 *      Author: Nuno Barros
 */

#include <thread>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <spdlog/spdlog.h>
#include <cerrno>
#include <zmq.hpp>
#include <string>
#include <mem_utils.h>


// reg 0 ised for reset
#define CONF_MEM_LOW   0xA0040000
#define CONF_MEM_HIGH  0xA004FFFF

#define GPIO_MEM_LOW   0xA0010000
#define GPIO_MEM_HIGH  0xA001FFFF

#define GPIO2_MEM_LOW   0xA0020000
#define GPIO2_MEM_HIGH  0xA002FFFF


#define CH_OFFSET      0x8

volatile std::atomic<bool> run;

int test_read_write(uintptr_t &addr_io, uintptr_t &addr_i)
{
  spdlog::info("--> Read test of CH0");
  spdlog::debug("Reading register for 5 s");
  for (size_t i = 0; i < 10; i++)
  {
    uint32_t val = cib::util::reg_read(addr_io+0*CH_OFFSET);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // -- try to write to the register, even if not a location that is useful
  spdlog::info("--> Write test of CH1");
  spdlog::debug("Writing to register of CH1 once");
  uint32_t val_write = 0xACDC0F;
  spdlog::trace("Writing value 0x{0:X}",val_write);
  cib::util::reg_write(addr_io+(1*CH_OFFSET), val_write);
  spdlog::debug("Reading on second GPIO the loopback value");

  uint32_t val_readback = cib::util::reg_read(addr_i+(0*CH_OFFSET));
  spdlog::trace("Read back 0x{0:X}",val_readback);
  if (val_readback != val_write)
  {
    spdlog::error("Readback value not the same as writing value ({0:X} <> {1:X})",val_readback,val_write);
    return -1;
  }
  spdlog::info("All tests were successful");
  return 0;
}

int main(int argc, char** argv)
{
  // setup spdlog
  run.store(true);
  // initiate spdlog
  spdlog::set_pattern("lbls : [%^%L%$] %v");
  spdlog::set_level(spdlog::level::trace); // Set global log level to debug

  spdlog::trace("Just testing a trace");
  spdlog::debug("Just testing a debug");

  spdlog::info("Mapping configuration module");
  int memfd = 0;
  uintptr_t vmem_conf = cib::util::map_phys_mem(memfd,CONF_MEM_LOW,CONF_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",vmem_conf);
  if (vmem_conf == 0x0)
  {
    spdlog::critical("Failed to map configuration memory. This is not going to end well.");
    return 255;
  }

  spdlog::info("Mapping GPIO_IO_0");
  uintptr_t vmem_gpio = cib::util::map_phys_mem(memfd,GPIO_MEM_LOW,GPIO_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",vmem_gpio);
  if (vmem_gpio == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    return 255;
  }

  spdlog::info("Mapping GPIO_I_0");
  uintptr_t vmem_gpio2 = cib::util::map_phys_mem(memfd,GPIO2_MEM_LOW,GPIO2_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",vmem_gpio2);
  if (vmem_gpio2 == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    return 255;
  }

  int res = test_read_write(vmem_gpio, vmem_gpio2);
  spdlog::info("Tests done");
  if (res != 0)
  {
    spdlog::critical("Somethign failed on memory mapped register control");
    return 0;
  }

  return 0;
}
