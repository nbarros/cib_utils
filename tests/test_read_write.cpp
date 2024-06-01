/*
 * test_read_write.cpp
 *
 *  Created on: Jun 1, 2024
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

extern "C"
{
#include <readline/readline.h>
#include <readline/history.h>
#include <unistd.h>
};

#include <cib_mem.h>

int test_read_write(uintptr_t &addr_io, uintptr_t &addr_i);

int test_read_write(uintptr_t &addr_io, uintptr_t &addr_i)
{
  spdlog::info("--> Read test of CH0");
  spdlog::debug("Reading register for 5 s");
  for (size_t i = 0; i < 10; i++)
  {
    uint32_t val = cib::util::reg_read(addr_io+0*GPIO_CH_OFFSET);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // -- try to write to the register, even if not a location that is useful
  spdlog::info("--> Write test of CH1");
  spdlog::debug("Writing to register of CH1 once");
  uint32_t val_write = 0xACDC0F;
  spdlog::trace("Writing value 0x{0:X}",val_write);
  cib::util::reg_write(addr_io+(1*GPIO_CH_OFFSET), val_write);
  spdlog::debug("Reading on second GPIO the loopback value");

  uint32_t val_readback = cib::util::reg_read(addr_i+(0*GPIO_CH_OFFSET));
  spdlog::trace("Read back 0x{0:X}",val_readback);
  if (val_readback != val_write)
  {
    spdlog::error("Readback value not the same as writing value ({0:X} <> {1:X})",val_readback,val_write);
    return -1;
  }
  spdlog::info("All tests were successful");
  return 0;
}

int g_mem_fd;
int main (int argc, char** argv)
{
  spdlog::set_pattern("cib : [%^%L%$] %v");
  spdlog::set_level(spdlog::level::info); // Set global log level to info

  spdlog::info("Mapping GPIO_I_0");
  uintptr_t p_addr_1 = GPIO_I_0_MEM_LOW;

  uintptr_t v_addr_1 = cib::util::map_phys_mem(g_mem_fd,GPIO_I_0_MEM_LOW,GPIO_I_0_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",v_addr_1);
  if (v_addr_1 == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    return 255;
  }

  spdlog::info("Mapping GPIO_I_1");
  uintptr_t p_addr_2 = GPIO_I_1_MEM_LOW;
  uintptr_t v_addr_2 = cib::util::map_phys_mem(g_mem_fd,GPIO_I_1_MEM_LOW,GPIO_I_1_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",v_addr_2);
  if (v_addr_2 == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    return 255;
  }

  // map the registers
    spdlog::info("Checking that all registers are mapped");

    int res = test_read_write(v_addr_1, v_addr_2);
    spdlog::info("Tests done");
    if (res != 0)
    {
      spdlog::critical("Something failed on memory mapped register control");
      return 0;
    }

  return 0;
}
