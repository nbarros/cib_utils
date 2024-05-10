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
#define CONF_CH_OFFSET 0x4

#define GPIO_MEM_LOW   0xA0010000
#define GPIO_MEM_HIGH  0xA001FFFF

#define GPIO2_MEM_LOW   0xA0020000
#define GPIO2_MEM_HIGH  0xA002FFFF

#define GPIO_CH_OFFSET      0x8

volatile std::atomic<bool> run;

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

// implement soeme extra commands
int set_motor_init_position(uintptr_t addr, uint32_t m1, uint32_t m2, uint32_t m3)
{
  spdlog::info("Setting initial position to [RNN800, RNN600, LSTAGE] = ({0}, {1}, {2})",m1,m2,m3);
  cib::util::reg_write_mask(addr+(CONF_CH_OFFSET*1),0,((1<<21)-1));
  std::this_thread::sleep_for(std::chrono::microseconds(10));
  cib::util::reg_write_mask(addr+(CONF_CH_OFFSET*2),0,((1<<21)-1));
  std::this_thread::sleep_for(std::chrono::microseconds(10));
  cib::util::reg_write_mask(addr+(CONF_CH_OFFSET*3),0,((1<<21)-1));
  std::this_thread::sleep_for(std::chrono::microseconds(10));


  // read the register back to be sure
  spdlog::info("Position set (readback [{},{},{}])",
               cib::util::reg_read(addr+(CONF_CH_OFFSET*1)),
               cib::util::reg_read(addr+(CONF_CH_OFFSET*2)),
               cib::util::reg_read(addr+(CONF_CH_OFFSET*3)));
  return 0;
}

int get_motor_init_position(uintptr_t addr)
{
  spdlog::info("Getting initial movement position from motors");
  spdlog::info("Position [RNN800, RNN600, LSTAGE] = [{},{},{}]",
               cib::util::reg_read(addr+(CONF_CH_OFFSET*1)),
               cib::util::reg_read(addr+(CONF_CH_OFFSET*2)),
               cib::util::reg_read(addr+(CONF_CH_OFFSET*3)));
   return 0;
}

int run_command(uintptr_t &memaddr, int argc, char** argv)
{
  if (argc< 1)
  {
    return 1;
  }

  std::string cmd(argv[0]);
  // check command request
  if (cmd == "exit")
  {
    return 255;
  }
  else if (cmd == "reset_pdts")
  {

  }
  else if (cmd == "set_motor_init")
  {
    if (argc != 4)
    {
      spdlog::warn("usage: set_motor_init pi1 pi2 pi3");
      return 0;
    }
    int32_t pi1 = std::strtoul(argv[1],NULL,0);
    int32_t pi2 = std::strtoul(argv[2],NULL,0);
    int32_t pi3 = std::strtoul(argv[3],NULL,0);
    spdlog::debug("Setting motor init position to [RNN800,RNN600,LSTAGE] = [{0},{1},{2}]",pi1,pi2,pi3);
    int res = set_motor_init_position(memaddr,pi1,pi2,pi3);
    if (res != 0)
    {
      spdlog::error("An unknown error was found");
      return 0;
    }
    spdlog::debug("Position set successfully!");
    return 0;
  }
  else if(cmd == "get_motor_init")
  {

  }
  else if (cmd == "set_laser_fire_width")
  {

  }
  else if (cmd == "set_laser_fire_width")
  {

  }
  else if (cmd == "set_laser_fire_period")
  {

  }
  else if (cmd == "fire_enable")
  {

  }

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
