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
}
#include <cib_mem.h>

volatile std::atomic<bool> run;

////////////////////////////////////////////////////////
///  prototypes
////////////////////////////////////////////////////////

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

int run_command(uintptr_t &memaddr,uintptr_t &memaddr_pdts, uintptr_t &memaddr_align, int argc, char** argv);
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
  uint32_t width = cib::util::reg_read(addr+(CONF_CH_OFFSET*17));
  width = width & cib::util::bitmask(0,15);

  uint32_t period = cib::util::reg_read(addr+(CONF_CH_OFFSET*16));
  width = width & cib::util::bitmask(0,23);
  spdlog::info("Current settings : width {0} ({1} us) period {2} ({3} us)",width,float(width)/16.,period,float(period)/16.0);
  return 0;
}

int set_align_laser_settings(uintptr_t &addr, const uint32_t width, const uint32_t period)
{
  spdlog::info("Stopping the alignment laser");
  set_align_laser_state(addr,0UL);

  spdlog::info("Setting alignment laser settings");

  uint32_t mask = cib::util::bitmask(0,15);
  spdlog::trace("Setting width with 0x{0:x} (mask 0x{1:X}",width, mask);
  cib::util::reg_write_mask(addr+(CONF_CH_OFFSET*17),width,mask);
  spdlog::trace("Width set");
  // now the period
  mask = cib::util::bitmask(0,23);
  spdlog::trace("Setting period with 0x{0:x} (mask 0x{1:X}",period, mask);
  cib::util::reg_write_mask(addr+(CONF_CH_OFFSET*16),period,mask);
  return 0;
}

int get_align_laser_state(uintptr_t &addr)
{
  spdlog::info("Getting alignment laser state");
  uint32_t state_word = cib::util::reg_read(addr+(CONF_CH_OFFSET*0));

  uint32_t mask = cib::util::bitmask(26,26);
  uint32_t state = state_word & mask;
  spdlog::info("Alignment laser state : {0}",(state==0)?0:1);
  return 0;
}

int set_align_laser_state(uintptr_t &addr, uint32_t state)
{
  spdlog::info("Setting alignment laser state to {0}",state);

  uint32_t state_word = cib::util::reg_read(addr+(CONF_CH_OFFSET*0));
  uint32_t mask = cib::util::bitmask(26,26);
  state = state <<26;
  cib::util::reg_write_mask(addr,state,mask);
  return 0;
}

int get_laser_settings(uintptr_t &addr)
{
  spdlog::info("Getting laser settings");
  uint32_t state_word = cib::util::reg_read(addr+(CONF_CH_OFFSET*0));
  uint32_t reg10 = cib::util::reg_read(addr+(CONF_CH_OFFSET*10));
  uint32_t reg11 = cib::util::reg_read(addr+(CONF_CH_OFFSET*11));
  uint32_t reg12 = cib::util::reg_read(addr+(CONF_CH_OFFSET*12));
  spdlog::trace("unpacking info");
  // now unpack the settings
  uint32_t mask = cib::util::bitmask(31,31);
  uint32_t fire_state = (state_word & mask) >> 31;

  mask = cib::util::bitmask(0,11);
  uint32_t fire_width = reg10 & mask;

  mask = cib::util::bitmask(0,23);
  uint32_t fire_period = reg12 & mask;

  mask = cib::util::bitmask(31,31);
  uint32_t qs_state = (reg10 & mask ) >> 31;

  mask = cib::util::bitmask(12,23);
  uint32_t qs_width = (reg10 & mask ) >> 12;

  mask = cib::util::bitmask(0,14);
  uint32_t qs_delay = (reg11 & mask );


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
  uint32_t mask = cib::util::bitmask(31,31);
  cib::util::reg_write_mask_offset(addr+(CONF_CH_OFFSET*0),state,mask,31);
  return 0;
}

int set_laser_qswitch_state(uintptr_t &addr, uint32_t state)
{
  uint32_t mask = cib::util::bitmask(31,31);
  cib::util::reg_write_mask_offset(addr+(CONF_CH_OFFSET*10),state,mask,31);
  return 0;
}


int set_laser_fire(uintptr_t &addr, const uint32_t width, const uint32_t period = 6250000)
{
  // first make sure to shut down the laser
  // keep track of the current laser state
  uint32_t state_word = cib::util::reg_read(addr+(CONF_CH_OFFSET*0));
  uint32_t state_mask = cib::util::bitmask(31,31);
  uint32_t state_cache = state_word & state_mask;
  set_laser_fire_state(addr,0UL);

  // take care of the width
  uint32_t mask = cib::util::bitmask(0,11);
  uint32_t w = width & mask;
  cib::util::reg_write_mask(addr+(CONF_CH_OFFSET*10),w,mask);

  //
  if (period != 6250000)
  {
    spdlog::warn("Overriding laser frequency. You better know what you're doing!!!!");
  }
  mask = cib::util::bitmask(0,23);
  uint32_t p = period & mask;
  cib::util::reg_write_mask(addr+(CONF_CH_OFFSET*12),p,mask);

  // reset the original state
  spdlog::debug("Resetting the original operation state");
  cib::util::reg_write_mask(addr+(CONF_CH_OFFSET*0),state_cache,state_mask);

  return 0;
}

int set_laser_qswitch(uintptr_t &addr, const uint32_t width, const uint32_t delay)
{
  // first make sure to shut down the laser
  // keep track of the current laser state
  uint32_t state_word = cib::util::reg_read(addr+(CONF_CH_OFFSET*0));
  uint32_t state_mask = cib::util::bitmask(31,31);
  uint32_t state_cache = state_word & state_mask;
  spdlog::debug("Stopping the laser while settings are changing");
  set_laser_fire_state(addr,0UL);

  spdlog::debug("Stopping the laser qswitch while settings are changing");
  uint32_t qs_word = cib::util::reg_read(addr+(CONF_CH_OFFSET*10));
  uint32_t qs_mask = cib::util::bitmask(31,31);
  uint32_t qs_cache = qs_word & qs_mask;

  set_laser_qswitch_state(addr,0UL);

  // take care of the width
  uint32_t mask = cib::util::bitmask(12,23);
  uint32_t w = (width << 12) & mask;
  cib::util::reg_write_mask(addr+(CONF_CH_OFFSET*10),w,mask);

  mask = cib::util::bitmask(0,14);
  uint32_t d = delay & mask;
  cib::util::reg_write_mask(addr+(CONF_CH_OFFSET*11),d,mask);

  // reset the original state
  spdlog::debug("Resetting the original qswitch state");
  cib::util::reg_write_mask(addr+(CONF_CH_OFFSET*10),qs_cache,qs_mask);
  spdlog::debug("Resetting the original fire state");
  cib::util::reg_write_mask(addr+(CONF_CH_OFFSET*0),state_cache,state_mask);

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
  spdlog::debug("Register value 0x{0:X} status 0x{1:X}",reg_val, (reg_val & mask));
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

int run_command(uintptr_t &memaddr,uintptr_t &memaddr_pdts, uintptr_t &memaddr_align, int argc, char** argv)
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
      res = get_motor_init_position(memaddr);
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
      int res = set_motor_init_position(memaddr,pi1,pi2,pi3);
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
      res = get_align_laser_settings(memaddr_align);
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
      res = set_align_laser_settings(memaddr_align,width,period);
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
      res = get_align_laser_state(memaddr_align);
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
      res = set_align_laser_state(memaddr_align,state);
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
      res = get_laser_settings(memaddr);
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

      res = set_laser_settings(memaddr,fire_state,fire_width,qs_state,qs_width,qs_delay);
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
      res = set_laser_settings(memaddr,fire_state,fire_width,qs_state,qs_width,qs_delay,fire_period);

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
      return get_laser_settings(memaddr);
    }
    else if (argc == 3)
    {
      spdlog::warn("Setting firing period. You better know what you are doing!!!");
      uint32_t width = std::strtoul(argv[1],NULL,0);
      uint32_t period = std::strtoul(argv[2],NULL,0);
      res = set_laser_fire(memaddr,width,period);
    }
    else if (argc == 2)
    {
      uint32_t width = std::strtoul(argv[1],NULL,0);
      res = set_laser_fire(memaddr,width);
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
      return get_laser_settings(memaddr);
    }
    else if (argc == 3)
    {
      uint32_t width = std::strtoul(argv[1],NULL,0);
      uint32_t delay = std::strtoul(argv[2],NULL,0);
      res = set_laser_qswitch(memaddr,width,delay);
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
      return get_laser_settings(memaddr);
    }
    else if (argc == 2)
    {
      uint32_t state = std::strtoul(argv[1],NULL,0);
      res = set_laser_fire_state(memaddr,state);
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
      return get_laser_settings(memaddr);
    }
    else if (argc == 2)
    {
      uint32_t state = std::strtoul(argv[1],NULL,0);
      res = set_laser_qswitch_state(memaddr,state);
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
      res = pdts_get_status(memaddr_pdts,stat,addr,ctl);
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
        res = pdts_set_addr(memaddr_pdts,addr);
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
    int res = pdts_reset(memaddr_pdts);
    if (res != 0)
    {
      spdlog::error("Failed to reset PDTS");
    }
    return 0;
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

  spdlog::info("Mapping GPIO_PDTS");
  uintptr_t vmem_pdts = cib::util::map_phys_mem(memfd,GPIO_PDTS_MEM_LOW,GPIO_PDTS_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",vmem_pdts);
  if (vmem_pdts == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    return 255;
  }

  spdlog::info("Mapping ALIGN_PDTS");
  uintptr_t vmem_align = cib::util::map_phys_mem(memfd,GPIO_ALIGN_MEM_LOW,GPIO_ALIGN_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",vmem_align);
  if (vmem_align == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    return 255;
  }


  spdlog::info("Mapping GPIO_I_0");
  uintptr_t vmem_gpio2 = cib::util::map_phys_mem(memfd,GPIO_I_0_MEM_LOW,GPIO_I_0_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",vmem_gpio2);
  if (vmem_gpio2 == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    return 255;
  }

//  int res = test_read_write(vmem_gpio, vmem_gpio2);
//  spdlog::info("Tests done");
//  if (res != 0)
//  {
//    spdlog::critical("Something failed on memory mapped register control");
//    return 0;
//  }
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
    if (count > 0) {
      char **cmd = new char*[count];
      cmd[0] = strtok(buf, delim);
      int i;
      for (i = 1; cmd[i-1] != NULL && i < count; i++) {
        cmd[i] = strtok(NULL, delim);
      }
      if (cmd[i-1] == NULL) i--;
      int ret = run_command(vmem_conf, vmem_pdts, vmem_align,i,cmd);
      delete [] cmd;
      if (ret == 255) return 0;
      if (ret != 0) return ret;
    } else {
      return 0;
    }
    free(buf);
  }
  return 0;
}
