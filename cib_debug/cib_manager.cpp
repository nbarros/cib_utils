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

extern "C"
{
#include <readline/readline.h>
#include <readline/history.h>
#include <unistd.h>
};

#include <cib_mem.h>
#include <AD5339.h>

volatile std::atomic<bool> run;

typedef struct mapped_mem
{
  uintptr_t p_addr;
  uintptr_t v_addr;
} mapped_mem;

typedef struct cib_mem
{
  mapped_mem config;
  mapped_mem gpio_pdts;
  mapped_mem gpio_align;
  mapped_mem gpio_laser;
  mapped_mem gpio_mon_0;
  mapped_mem gpio_mon_1;
} cib_mem;


cib_mem g_cib_mem;

////////////////////////////////////////////////////////
///  prototypes
////////////////////////////////////////////////////////

void clear_memory();
int setup_dac(cib::i2c::AD5339 &dac);
int pdts_reset(uintptr_t &addr);
int pdts_get_status(uintptr_t &addr, uint16_t &pdts_stat, uint16_t &pdts_addr, uint16_t &pdts_ctl);
int pdts_set_addr(uintptr_t &addr,uint16_t pdts_addr);
int get_align_laser_state(uintptr_t &addr);
int test_read_write(uintptr_t &addr_io, uintptr_t &addr_i);
int set_motor_init_position(uintptr_t addr, uint32_t m1, uint32_t m2, uint32_t m3);
int get_motor_init_position(uintptr_t addr);
int get_align_laser_settings(uintptr_t &addr);
int set_align_laser_settings(uintptr_t &addr, const uint32_t width, const uint32_t period);
int get_align_laser_state(uintptr_t &addr);
int set_align_laser_state(uintptr_t &addr, uint32_t state);
int get_laser_settings(uintptr_t &addr);
int set_laser_fire_state(uintptr_t &addr, uint32_t state);
int set_laser_qswitch_state(uintptr_t &addr, uint32_t state);
int set_laser_fire(uintptr_t &addr, const uint32_t width, const uint32_t period);
int set_laser_qswitch(uintptr_t &addr, const uint32_t width, const uint32_t delay);
int set_laser_settings(uintptr_t &addr,
                       uint32_t fire_state, uint32_t fire_width,
                       uint32_t qs_state,  uint32_t qs_width, uint32_t qs_delay, uint32_t fire_period);

int run_command(int argc, char** argv);
void print_help();



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


void clear_memory()
{
  spdlog::info("Unmapping the register memory");
  spdlog::debug("* config");
  int ret = cib::util::unmap_mem(g_cib_mem.config.v_addr,PAGE_SIZE);
  if (ret != 0)
  {
    spdlog::error("Failed to unmap config");
  }
  ret = cib::util::unmap_mem(g_cib_mem.gpio_laser.v_addr,PAGE_SIZE);
  if (ret != 0)
  {
    spdlog::error("Failed to unmap laser");
  }
  ret = cib::util::unmap_mem(g_cib_mem.gpio_align.v_addr,PAGE_SIZE);
  if (ret != 0)
  {
    spdlog::error("Failed to unmap align");
  }
  ret = cib::util::unmap_mem(g_cib_mem.gpio_pdts.v_addr,PAGE_SIZE);
  if (ret != 0)
  {
    spdlog::error("Failed to unmap pdts");
  }
  ret = cib::util::unmap_mem(g_cib_mem.gpio_mon_0.v_addr,PAGE_SIZE);
  if (ret != 0)
  {
    spdlog::error("Failed to unmap mon_0");
  }
  ret = cib::util::unmap_mem(g_cib_mem.gpio_mon_1.v_addr,PAGE_SIZE);
  if (ret != 0)
  {
    spdlog::error("Failed to unmap mon_1");
  }
 }

int setup_dac(cib::i2c::AD5339 &dac)
{
  spdlog::debug("Configuring the DAC to the appropriate settings [bus = 7; addr = 0xd]");
  int res = dac.set_bus(7);
  if (res != CIB_I2C_OK)
  {
    spdlog::critical("Failed to set bus number. Returned 0x{:X} ({})",res,cib::i2c::strerror(res));
    return res;
  }
  res = dac.set_dev_number(0xd);
  if (res != CIB_I2C_OK)
  {
    spdlog::critical("Failed to set dev number. Returned 0x{:X} ({})",res,cib::i2c::strerror(res));
    return res;
  }
  return CIB_I2C_OK;
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


int get_align_laser_settings(uintptr_t &addr)
{
  spdlog::info("Getting alignment laser settings");

  uintptr_t maddr = addr + (GPIO_CH_OFFSET*1);
  uint32_t mask = cib::util::bitmask(15,0);
  uint32_t width = cib::util::reg_read(maddr);
  width = width & mask;

  // the period is located in channel 0
  maddr =  addr + (GPIO_CH_OFFSET*0);
  uint32_t period = cib::util::reg_read(maddr);
  mask = cib::util::bitmask(23,0);
  period = period & mask;
  spdlog::info("Current settings : width {0} ({1} us) period {2} ({3} us)",width,float(width)/16.,period,float(period)/16.0);
  return 0;
}

int set_align_laser_settings(uintptr_t &addr, const uint32_t width, const uint32_t period)
{
  spdlog::info("Stopping the alignment laser");
  set_align_laser_state(addr,0UL);

  spdlog::info("Setting alignment laser settings");
  uintptr_t maddr = addr + (GPIO_CH_OFFSET*1);

  // ch 1: [15:0]
  uint32_t mask = cib::util::bitmask(15,0);
  spdlog::trace("Setting width with 0x{0:x} (mask 0x{1:X})",width, mask);
  cib::util::reg_write_mask(maddr,width,mask);
  spdlog::trace("Width set");
  // now the period

  maddr = addr + (GPIO_CH_OFFSET*0);
  mask = cib::util::bitmask(23,0);
  spdlog::trace("Setting period with 0x{0:x} (mask 0x{1:X}",period, mask);
  cib::util::reg_write_mask(maddr,period,mask);
  return 0;
}

int get_align_laser_state(uintptr_t &addr)
{
  spdlog::info("Getting alignment laser state");
  uintptr_t maddr = addr + (GPIO_CH_OFFSET*0);
  uint32_t state_word = cib::util::reg_read(maddr);

  uint32_t mask = cib::util::bitmask(31,31);
  uint32_t state = state_word & mask;
  spdlog::info("Alignment laser state : {0}",(state==0)?0:1);
  return 0;
}

int set_align_laser_state(uintptr_t &addr, uint32_t state)
{
  spdlog::info("Setting alignment laser state to {0}",state);
  uintptr_t maddr = addr + (GPIO_CH_OFFSET*0);

  uint32_t state_word = cib::util::reg_read(maddr);
  uint32_t mask = cib::util::bitmask(31,31);
  cib::util::reg_write_mask_offset(maddr,state,mask,31);
  return 0;
}

int get_laser_settings(uintptr_t &addr)
{
  spdlog::info("Getting laser settings");

  // in fact there are only 2 registers to be read
  // FIXME: Make qswitch width independet
  //        Make period settable

  uint32_t maddr = addr +(GPIO_CH_OFFSET*0);

  uint32_t fire_word = cib::util::reg_read(maddr);
  maddr = addr +(GPIO_CH_OFFSET*1);
  uint32_t qswitch_word = cib::util::reg_read(maddr);

  spdlog::trace("unpacking info");
  // now unpack the settings
  uint32_t mask = cib::util::bitmask(31,31);
  uint32_t fire_state = (fire_word & mask) >> 31;

  mask = cib::util::bitmask(11,0);
  uint32_t fire_width = fire_word & mask;

  uint32_t fire_period = 6250000;
//  mask = cib::util::bitmask(23,0);
//  uint32_t fire_period = reg12 & mask;

  mask = cib::util::bitmask(31,31);
  uint32_t qs_state = (qswitch_word & mask ) >> 31;

//  mask = cib::util::bitmask(15,0);
//  uint32_t qs_width = (reg10 & mask ) >> 12;

  uint32_t qs_width = fire_width;

  mask = cib::util::bitmask(14,0);
  uint32_t qs_delay = (qswitch_word & mask );


  spdlog::info("Laser setttings :\n"
      "FIRE:\n"
      "\t Enabled {0}\n"
      "\t Width   {1} ({2} us)\n"
      "\t Period  {3} ({4} us)\n"
      "QSWITCH:\n"
      "\t Enabled {5}\n"
      "\t Width   {6} ({7} us)\n"
      "\t delay   {8} ({9} us)\n"
      ,fire_state, fire_width, float(fire_width)/16.0,fire_period,float(fire_period)/16.0
      ,qs_state, qs_width,float(qs_width)/16.0,qs_delay,float(qs_delay)/16.0);
  return 0;
}


int set_laser_fire_state(uintptr_t &addr, uint32_t state)
{
  uint32_t maddr = addr +(GPIO_CH_OFFSET*0);
  uint32_t mask = cib::util::bitmask(31,31);
  cib::util::reg_write_mask_offset(maddr,state,mask,31);
  return 0;
}

int set_laser_qswitch_state(uintptr_t &addr, uint32_t state)
{
  uint32_t maddr = addr +(GPIO_CH_OFFSET*0);
  uint32_t mask = cib::util::bitmask(31,31);
  cib::util::reg_write_mask_offset(maddr,state,mask,31);
  return 0;
}


int set_laser_fire(uintptr_t &addr, const uint32_t width, const uint32_t period = 6250000)
{
  // first make sure to shut down the laser
  // keep track of the current laser state
  uint32_t maddr = addr +(GPIO_CH_OFFSET*0);

  uint32_t state_word = cib::util::reg_read(maddr);
  uint32_t state_mask = cib::util::bitmask(31,31);
  uint32_t state_cache = state_word & state_mask;
  set_laser_fire_state(addr,0UL);

  // take care of the width
  uint32_t mask = cib::util::bitmask(11,0);
  uint32_t w = width & mask;
  cib::util::reg_write_mask(maddr,w,mask);

  //
//  if (period != 6250000)
//  {
//    //spdlog::warn("Overriding laser frequency. You better know what you're doing!!!!");
//  }
//  mask = cib::util::bitmask(23,0);
//  uint32_t p = period & mask;
//  cib::util::reg_write_mask(addr+(CONF_CH_OFFSET*12),p,mask);

  // reset the original state
  spdlog::debug("Resetting the original operation state");
  cib::util::reg_write_mask(maddr,state_cache,state_mask);

  return 0;
}

int set_laser_qswitch(uintptr_t &addr, const uint32_t width, const uint32_t delay)
{
  // first make sure to shut down the laser
  // keep track of the current laser state
  uint32_t maddr = addr +(GPIO_CH_OFFSET*0);
  uint32_t state_word = cib::util::reg_read(maddr);
  uint32_t state_mask = cib::util::bitmask(31,31);
  uint32_t state_cache = state_word & state_mask;
  spdlog::debug("Stopping the laser while settings are changing");
  set_laser_fire_state(addr,0UL);

  spdlog::debug("Stopping the laser qswitch while settings are changing");
  maddr = addr +(GPIO_CH_OFFSET*1);
  uint32_t qs_word = cib::util::reg_read(maddr);
  uint32_t qs_mask = cib::util::bitmask(31,31);
  uint32_t qs_cache = qs_word & qs_mask;

  set_laser_qswitch_state(addr,0UL);

  // take care of the width
  maddr = addr +(GPIO_CH_OFFSET*0);
  uint32_t mask = cib::util::bitmask(11,0);
  cib::util::reg_write_mask(maddr,width,mask);

  maddr = addr +(GPIO_CH_OFFSET*1);
  mask = cib::util::bitmask(14,0);
  uint32_t d = delay & mask;
  cib::util::reg_write_mask(maddr,d,mask);

  // reset the original state
  spdlog::debug("Resetting the original qswitch state");
  maddr = addr +(GPIO_CH_OFFSET*1);
  cib::util::reg_write_mask(maddr,qs_cache,qs_mask);
  spdlog::debug("Resetting the original fire state");
  maddr = addr +(GPIO_CH_OFFSET*0);
  cib::util::reg_write_mask(maddr,state_cache,state_mask);

  return 0;
}


int set_laser_settings(uintptr_t &addr,
                       uint32_t fire_state, uint32_t fire_width,
                       uint32_t qs_state,  uint32_t qs_width, uint32_t qs_delay, uint32_t fire_period = 6250000)
{
  spdlog::info("Configuring the laser settings");
  // first force everything to off
  spdlog::info("Shutting down the laser states");

  set_laser_fire_state(addr,0UL);
  set_laser_qswitch_state(addr,0UL);

  spdlog::debug("Configuring fire");
  set_laser_fire(addr, fire_width, fire_period);

  spdlog::debug("Configuring qswitch");
  set_laser_fire(addr, qs_width, qs_delay);

  spdlog::debug("Setting the states");

  // first fire
  set_laser_fire_state(addr,fire_state);
  set_laser_qswitch_state(addr,qs_state);
  spdlog::debug("Laser configured to your desires");

  return 0;
}

int pdts_reset(uintptr_t &addr)
{
  spdlog::info("Resetting PDTS");
  uintptr_t memaddr = addr+(GPIO_CH_OFFSET*1);
  uint32_t mask = cib::util::bitmask(31,31);
  spdlog::trace("Setting reset high");
  cib::util::reg_write_mask_offset(memaddr,0x1,mask,31);
  spdlog::trace("Sleeping for 10 us");
  std::this_thread::sleep_for(std::chrono::microseconds(10));
  spdlog::trace("Setting reset low");
  cib::util::reg_write_mask(memaddr,0x0,mask);
  return 0;
}


int pdts_get_status(uintptr_t &addr, uint16_t &pdts_stat, uint16_t &pdts_addr, uint16_t &pdts_ctl)
{
  // information in the pdts status register:
  // [0:3] : status
  // [4:11] : ctrl
  // [12:27] : address

  // first channel of the GPIO is the read one
  uintptr_t memaddr = addr + (GPIO_CH_OFFSET*0);

  spdlog::debug("Reading memory address 0x{0:X}",memaddr);
  uint32_t reg_val = cib::util::reg_read(addr+(GPIO_CH_OFFSET*0));

  uint32_t mask = cib::util::bitmask(0,3);

  spdlog::debug("Register value 0x{0:X} mask 0x{1:X} status 0x{2:X}",reg_val, mask, (reg_val & mask));
  spdlog::info("PDTS STATUS : 0x{0:X}",(reg_val & mask));
  pdts_stat = (reg_val & mask);

  spdlog::debug("Checking pdts address");
  mask = cib::util::bitmask(12,27);
  spdlog::info("PDTS ADDR : 0x{0:X}",((reg_val & mask)>>12));
  pdts_addr = ((reg_val & mask)>>12);

  spdlog::debug("Checking pdts ctrl register");
  mask = cib::util::bitmask(4,11);
  spdlog::info("PDTS CTRL : 0x{0:X}",((reg_val & mask)>>4));
  pdts_ctl = ((reg_val & mask)>>4);

  return 0;
}

int pdts_set_addr(uintptr_t &addr,uint16_t pdts_addr)
{
  spdlog::debug("Setting pdts address to 0x{0:X}",pdts_addr);

  // first channel of the GPIO is the read one
  uintptr_t memaddr = addr + (GPIO_CH_OFFSET*1);
  spdlog::trace("Setting pdts address in memory address 0x{0:X}",memaddr);
  uint32_t mask = cib::util::bitmask(0,15);
  cib::util::reg_write_mask(memaddr,pdts_addr,mask);

  // after this we should perhaps sleep for a little and recheck
  spdlog::trace("Resetting the endpoint");
  pdts_reset(addr);
  spdlog::debug("Confirming pdts address change");
  uint16_t st, ad, ct;
  pdts_get_status(addr,st,ad,ct);

  if (ad == pdts_addr)
  {
    spdlog::debug("Address change confirmed");
  }
  else
  {
    spdlog::warn("Mismatching addresses: set 0x{0:X} readback 0x{1:X}",pdts_addr,ad);
  }
  // alternative, for now
//  uint32_t val = pdts_addr;
//  val = val << 16;
//  cib::util::reg_write(addr+(CONF_CH_OFFSET*13),val);

  return 0;
}

int run_command(int argc, char** argv)
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
  else if (cmd == "help")
  {
    print_help();
    return 0;
  }
  else if (cmd == "motor_init")
  {
    int res = 0;
    if (argc == 1)
    {
      // just want to do a readout
      res = get_motor_init_position(g_cib_mem.config.v_addr);
    }
    if ((argc != 4))
    {
      spdlog::warn("usage: motor_init [pi1 pi2 pi3]");
      return 0;
    }
    else
    {
      int32_t pi1 = std::strtol(argv[1],NULL,0);
      int32_t pi2 = std::strtol(argv[2],NULL,0);
      int32_t pi3 = std::strtol(argv[3],NULL,0);
      spdlog::debug("Setting motor init position to [RNN800,RNN600,LSTAGE] = [{0},{1},{2}]",pi1,pi2,pi3);
      int res = set_motor_init_position(g_cib_mem.config.v_addr,pi1,pi2,pi3);
    }
    if (res != 0)
    {
      spdlog::error("An unknown error was found");
    }
    return 0;
  }
  else if(cmd == "align_laser")
  {
    int res = 0;
    // arguments are width and period
    if (argc == 1)
    {
      // just show the existing settings
      res = get_align_laser_settings(g_cib_mem.gpio_align.v_addr);
    }
    else if (argc !=3)
    {
      spdlog::warn("usage: align_laser  width period (in 16 ns units)");
      return 0;
    }
    else
    {
      uint32_t width = std::strtoul(argv[1],NULL,0);
      uint32_t period = std::strtoul(argv[2],NULL,0);
      spdlog::debug("Setting the alignment laser settings to w {0} p {1}",width,period);
      res = set_align_laser_settings(g_cib_mem.gpio_align.v_addr,width,period);
    }
    if (res != 0)
    {
      spdlog::error("Error dealing with the alignment laser");
    }
    return 0;
  }
  else if (cmd == "align_enable")
  {
    int res = 0;
    if (argc == 1)
    {
      // just show the current state
      res = get_align_laser_state(g_cib_mem.gpio_align.v_addr);
    }
    else if (argc !=2)
    {
      spdlog::warn("usage: align_state state (1: ON; 0: OFF)");
      return 0;
    }
    else
    {
      uint32_t state = std::strtoul(argv[1],NULL,0);
      spdlog::debug("Setting the alignment laser state to {0}",state);
      res = set_align_laser_state(g_cib_mem.gpio_align.v_addr,state);
    }
    if (res != 0)
    {
      spdlog::error("Error dealing with the alignment laser state");
    }
    return 0;
  }
  else if (cmd == "laser_config")
  {
    int res = 0;
    if (argc == 1)
    {
      // just show the current state
      res = get_laser_settings(g_cib_mem.gpio_laser.v_addr);
    }
    // this has quite a few arguments
    // fire width, fire period, qswitch width, qswitch period
    else if (argc == 6)
    {
      uint32_t fire_state = std::strtoul(argv[1],NULL,0);
      uint32_t fire_width = std::strtoul(argv[2],NULL,0);
      uint32_t qs_state = std::strtoul(argv[3],NULL,0);
      uint32_t qs_width = std::strtoul(argv[4],NULL,0);
      uint32_t qs_delay = std::strtoul(argv[5],NULL,0);
      spdlog::info("Laser setttings to be set:\n"
          "FIRE:\n"
          "\t Enabled {0}\n"
          "\t Width   {1} ({2} us)\n"
          "QSWITCH:\n"
          "\t Enabled {3}\n"
          "\t Width   {4} ({5} us)\n"
          "\t delay   {6} ({7} us)\n"
          ,fire_state, fire_width, float(fire_width)/16.0
          ,qs_state, qs_width,float(qs_width)/16.0,qs_delay,float(qs_delay)/16.0);

      res = set_laser_settings(g_cib_mem.gpio_laser.v_addr,fire_state,fire_width,qs_state,qs_width,qs_delay);
    }
    else if (argc == 7)
    {
      uint32_t fire_state = std::strtoul(argv[1],NULL,0);
      uint32_t fire_width = std::strtoul(argv[2],NULL,0);
      uint32_t fire_period = std::strtoul(argv[3],NULL,0);
      uint32_t qs_state = std::strtoul(argv[4],NULL,0);
      uint32_t qs_width = std::strtoul(argv[5],NULL,0);
      uint32_t qs_delay = std::strtoul(argv[6],NULL,0);
      spdlog::warn("Overriding laser period!!!");
      spdlog::info("Laser setttings to be set:\n"
          "FIRE:\n"
          "\t Enabled {0}\n"
          "\t Width   {1} ({2} us)\n"
          "\t Period  {3} ({4} us)\n"
          "QSWITCH:\n"
          "\t Enabled {5}\n"
          "\t Width   {6} ({7} us)\n"
          "\t delay   {8} ({9} us)\n"
          ,fire_state, fire_width, float(fire_width)/16.0,fire_period,float(fire_period)/16.0
          ,qs_state, qs_width,float(qs_width)/16.0,qs_delay,float(qs_delay)/16.0);
      res = set_laser_settings(g_cib_mem.gpio_laser.v_addr,fire_state,fire_width,qs_state,qs_width,qs_delay,fire_period);

    }
    else
    {
      spdlog::warn("usage: laser_config fire_state fire_width [fire_period] qs_state qs_width qs_delay\n"
          "        *_state settings are (1 : ON; 0: OFF)\n"
          "        other settings are units of 16 ns\n"
          "        fire_period is optional and should not really be changed\n"
      );
    }
    if (res != 0)
    {
      spdlog::error("Error dealing with the laser config");
    }
    return 0;

  }
  else if (cmd == "fire_config")
  {
    int res = 0;
    if (argc == 1)
    {
      // just return the whole laser settings
      return get_laser_settings(g_cib_mem.gpio_laser.v_addr);
    }
    else if (argc == 3)
    {
      spdlog::warn("Setting firing period. You better know what you are doing!!!");
      uint32_t width = std::strtoul(argv[1],NULL,0);
      uint32_t period = std::strtoul(argv[2],NULL,0);
      res = set_laser_fire(g_cib_mem.gpio_laser.v_addr,width,period);
    }
    else if (argc == 2)
    {
      uint32_t width = std::strtoul(argv[1],NULL,0);
      res = set_laser_fire(g_cib_mem.gpio_laser.v_addr,width);
    }
    else
    {
      spdlog::warn("usage: fire_config width [period] (in 16 ns clock cycles)\n Note: be VERY careful with changing the period from the nominal 6250000.");
    }
    if (res != 0)
    {
      spdlog::error("Failed to execute command");
    }
    return 0;
  }
  else if (cmd == "qswitch_config")
  {
    int res = 0;
    if (argc == 1)
    {
      // just return the whole laser settings
      return get_laser_settings(g_cib_mem.gpio_laser.v_addr);
    }
    else if (argc == 3)
    {
      uint32_t width = std::strtoul(argv[1],NULL,0);
      uint32_t delay = std::strtoul(argv[2],NULL,0);
      res = set_laser_qswitch(g_cib_mem.gpio_laser.v_addr,width,delay);
    }
    else
    {
      spdlog::warn("usage: qswitch_config width delay ( in 16 ns units)");
    }
    if (res != 0)
    {
      spdlog::error("Failed to execute command");
    }
    return 0;

  }
  else if (cmd == "fire_enable")
  {
    int res = 0;
    if (argc == 1)
    {
      // just return the whole laser settings
      return get_laser_settings(g_cib_mem.gpio_laser.v_addr);
    }
    else if (argc == 2)
    {
      uint32_t state = std::strtoul(argv[1],NULL,0);
      res = set_laser_fire_state(g_cib_mem.gpio_laser.v_addr,state);
    }
    else
    {
      spdlog::warn("usage: fire_enable state (1: ON; 0:OFF)");
    }
    if (res != 0)
    {
      spdlog::error("Failed to execute command");
    }
    return 0;
  }
  else if (cmd == "qswitch_enable")
  {
    int res = 0;
    if (argc == 1)
    {
      // just return the whole laser settings
      return get_laser_settings(g_cib_mem.gpio_laser.v_addr);
    }
    else if (argc == 2)
    {
      uint32_t state = std::strtoul(argv[1],NULL,0);
      res = set_laser_qswitch_state(g_cib_mem.gpio_laser.v_addr,state);
    }
    else
    {
      spdlog::warn("usage: qswitch_enable state (1: ON; 0:OFF)");
    }
    return 0;
  }
  else if (cmd == "lbls")
  {
    spdlog::debug("Configuring lbls communication system");
    //int res =
    if (argc == 1)
    {
      // just read the settings

    }
    return 0;
  }
  else if (cmd == "pdts")
  {
    int res = 0;
    if (argc == 1)
    {
      spdlog::debug("Querying the pdts status");
      uint16_t stat, addr, ctl;
      res = pdts_get_status(g_cib_mem.gpio_pdts.v_addr,stat,addr,ctl);
      if (res != 0)
      {
        spdlog::error("Failed to get PDTS status");
      }
    }
    else if (argc == 3)
    {
      spdlog::debug("Got the following args {0},{1},{2}",argv[0],argv[1],argv[2]);
      if (std::string(argv[1]) == std::string("addr"))
      {
        uint16_t addr = std::strtol(argv[2],NULL,0);
        spdlog::info("Setting address to 0x{0:x}",addr);
        res = pdts_set_addr(g_cib_mem.gpio_pdts.v_addr,addr);
        if (res != 0)
        {
          spdlog::error("Failed to set address");
        }
      }
      else
      {
        spdlog::error("unknown subcommand [|{0}| |{1}|]",argv[1],argv[2]);
      }
    }
    else
    {
      spdlog::warn("Usage: pdts [addr <addr>]");
    }
  //    int res = check_pdts(memaddr);
  //    if (res != 0)
  //    {
  //      spdlog::error("Failed to reset PDTS");
  //    }
    return 0;
  }
  else if (cmd == "pdts_reset")
  {
    spdlog::info("Resetting the PDTS system");
    int res = pdts_reset(g_cib_mem.gpio_pdts.v_addr);
    if (res != 0)
    {
      spdlog::error("Failed to reset PDTS");
    }
    return 0;
  }
  else if (cmd == "dac")
  {

    if (argc == 1)
    {
      spdlog::info("Querying the DAC level");

    }

  }
  else
  {
    spdlog::error("Unknown command");
    return 0;
  }
  return 0;
}

void print_help()
{
  spdlog::info("Available commands (note, commands without arguments print current settings):");
  spdlog::info("  reset_pdts");
  spdlog::info("    Resets the PDTS system");
  spdlog::info("  pdts [addr <addr>]");
  spdlog::info("    Gets the current state of the PDTS system");
  spdlog::info("  lbls [state width]");
  spdlog::info("    Configure the LBLS trigger");
  spdlog::info("  align_laser [width period]");
  spdlog::info("    Config alignment laser");
  spdlog::info("  align_enable [state]");
  spdlog::info("    Enable/disable alignment laser");
  spdlog::info("  motor_init [pi_1 pi_2 pi_3]");
  spdlog::info("    Config the initial position of the motor (in the FPGA)");
  spdlog::info("  laser_config [laser_config fire_state fire_width [fire_period] qs_state qs_width qs_delay]");
  spdlog::info("    Configure the laser system in a single command. WARNING: Careful setting the fire_period");
  spdlog::info("  fire_enable [state]");
  spdlog::info("    Enable/disable laser FIRE ");
  spdlog::info("  fire_config [state width [period]]");
  spdlog::info("    Configures the FIRE part of the laser");
  spdlog::info("    WARNING: Do not edit the period, unless you know what you are doing");
  spdlog::info("  qswitch_enable [state]");
  spdlog::info("    Enable/disable laser QSWITCH ");
  spdlog::info("  qswitch_config [state width delay]");
  spdlog::info("    Configures the QSWITCH part of the laser");
  spdlog::info("  help");
  spdlog::info("    Print this help");
  spdlog::info("  exit");
  spdlog::info("    Closes the command interface");
  spdlog::info("");
  spdlog::info("Note: widths, periods and delays are all in 16 ns units. States are 1 or 0");

}

int main(int argc, char** argv)
{
  // setup spdlog
  run.store(true);
  // initiate spdlog
  spdlog::set_pattern("cib : [%^%L%$] %v");
  spdlog::set_level(spdlog::level::trace); // Set global log level to debug

  spdlog::trace("Just testing a trace");
  spdlog::debug("Just testing a debug");

  spdlog::info("Mapping configuration module");
  int memfd = 0;
  g_cib_mem.config.p_addr = CONF_MEM_LOW;
  g_cib_mem.config.v_addr = cib::util::map_phys_mem(memfd,CONF_MEM_LOW,CONF_MEM_HIGH);
  //uintptr_t vmem_conf = cib::util::map_phys_mem(memfd,CONF_MEM_LOW,CONF_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",g_cib_mem.config.v_addr);
  if (g_cib_mem.config.v_addr == 0x0)
  {
    spdlog::critical("Failed to map configuration memory. This is not going to end well.");
    clear_memory();
    return 255;
  }

  spdlog::info("Mapping GPIO_PDTS");
  g_cib_mem.gpio_pdts.p_addr = GPIO_PDTS_MEM_LOW;
  g_cib_mem.gpio_pdts.v_addr = cib::util::map_phys_mem(memfd,GPIO_PDTS_MEM_LOW,GPIO_PDTS_MEM_HIGH);
  //uintptr_t vmem_pdts = cib::util::map_phys_mem(memfd,GPIO_PDTS_MEM_LOW,GPIO_PDTS_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",g_cib_mem.gpio_pdts.v_addr);
  if (g_cib_mem.gpio_pdts.v_addr == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    clear_memory();
    return 255;
  }

  spdlog::info("Mapping GPIO_ALIGN");
  g_cib_mem.gpio_align.p_addr = GPIO_ALIGN_MEM_LOW;
  g_cib_mem.gpio_align.v_addr = cib::util::map_phys_mem(memfd,GPIO_ALIGN_MEM_LOW,GPIO_ALIGN_MEM_HIGH);
//  uintptr_t vmem_align = cib::util::map_phys_mem(memfd,GPIO_ALIGN_MEM_LOW,GPIO_ALIGN_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",g_cib_mem.gpio_align.v_addr);
  if (g_cib_mem.gpio_align.v_addr == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    clear_memory();
    return 255;
  }

  spdlog::info("Mapping GPIO_LASER");
  g_cib_mem.gpio_laser.p_addr = GPIO_LASER_MEM_LOW;
  g_cib_mem.gpio_laser.v_addr = cib::util::map_phys_mem(memfd,GPIO_LASER_MEM_LOW,GPIO_LASER_MEM_HIGH);
//  uintptr_t vmem_align = cib::util::map_phys_mem(memfd,GPIO_ALIGN_MEM_LOW,GPIO_ALIGN_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",g_cib_mem.gpio_laser.v_addr);
  if (g_cib_mem.gpio_laser.v_addr == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    clear_memory();
    return 255;
  }

  spdlog::info("Mapping GPIO_I_0");
  g_cib_mem.gpio_mon_0.p_addr = GPIO_I_0_MEM_LOW;

  g_cib_mem.gpio_mon_0.v_addr = cib::util::map_phys_mem(memfd,GPIO_I_0_MEM_LOW,GPIO_I_0_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",g_cib_mem.gpio_mon_0.v_addr);
  if (g_cib_mem.gpio_mon_0.v_addr == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    clear_memory();
    return 255;
  }

  spdlog::info("Mapping GPIO_I_1");
  g_cib_mem.gpio_mon_1.p_addr = GPIO_I_1_MEM_LOW;

  g_cib_mem.gpio_mon_1.v_addr = cib::util::map_phys_mem(memfd,GPIO_I_1_MEM_LOW,GPIO_I_1_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",g_cib_mem.gpio_mon_1.v_addr);
  if (g_cib_mem.gpio_mon_1.v_addr == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    clear_memory();
    return 255;
  }

//  spdlog::info("Checking that all registers are mapped");

//  int res = test_read_write(vmem_gpio, vmem_gpio2);
//  spdlog::info("Tests done");
//  if (res != 0)
//  {
//    spdlog::critical("Something failed on memory mapped register control");
//    return 0;
//  }

  spdlog::info("Instantiating the DAC");
  cib::i2c::AD5339 dac;
  int ret = setup_dac(dac);
  if (ret != CIB_I2C_OK)
  {
    spdlog::error("Failed to set up dac. Got code 0x{0:X} {1}",ret,cib::i2c::strerror(ret));
    clear_memory();
    return 0;
  }
  // by default set to the appropriate settings
  print_help();

  // -- now start the real work
  char* buf;
  while ((buf = readline(">> ")) != nullptr)
  {
    if (strlen(buf) > 0) {
      add_history(buf);
    } else {
      free(buf);
      continue;
    }
    char *delim = (char*)" ";
    int count = 1;
    char *ptr = buf;
    while((ptr = strchr(ptr, delim[0])) != NULL) {
      count++;
      ptr++;
    }
    if (count > 0)
    {
      char **cmd = new char*[count];
      cmd[0] = strtok(buf, delim);
      int i;
      for (i = 1; cmd[i-1] != NULL && i < count; i++)
      {
        cmd[i] = strtok(NULL, delim);
      }
      if (cmd[i-1] == NULL) i--;
      int ret = run_command(i,cmd);
      delete [] cmd;
      if (ret == 255)
      {
        clear_memory();
        close(memfd);
        return 0;
      }
      if (ret != 0)
      {
        clear_memory();
        close(memfd);
        return ret;
      }
    }
    else
    {
      clear_memory();
      close(memfd);
      return 0;
    }
    free(buf);
  }
  close(memfd);
  return 0;
}
