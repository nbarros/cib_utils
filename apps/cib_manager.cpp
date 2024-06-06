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
#include <cib_data_fmt.h>
#include <AD5339.h>

volatile std::atomic<bool> run;

using cib::cib_mem_t;
using cib::mapped_mem_t;
using cib::motor_t;

cib_mem_t g_cib_mem;
cib::i2c::AD5339 *g_dac;
int g_mem_fd;

////////////////////////////////////////////////////////
///  prototypes
////////////////////////////////////////////////////////

int map_memory();
void clear_memory();
// DAC for PDI
int setup_dac(cib::i2c::AD5339 &dac);
// lbls control
int lbls_set_width(uintptr_t &addr, uint32_t &width);
int lbls_set_state(uintptr_t &addr, uint32_t state);
int lbls_get_status(uintptr_t &addr, uint32_t &state, uint32_t &width);
// system reset (global reset)
int sys_reset(uintptr_t &addr);
// internal pulser
int trigger_pulser_get_state(uintptr_t &addr, uint32_t &state);
int trigger_pulser_set_state(uintptr_t &addr, uint32_t &state);
// external trigger
int trigger_ext_get_state(uintptr_t &addr, uint32_t &state);
int trigger_ext_set_state(uintptr_t &addr, uint32_t &state);
// -- shutter
int shutter_set_state(uintptr_t &addr, uint32_t &state);
int shutter_get_state(uintptr_t &addr, uint32_t &state);
// pdts
int pdts_reset(uintptr_t &addr);
int pdts_reset_domain(uintptr_t &addr);
int pdts_get_status(uintptr_t &addr, uint16_t &pdts_stat, uint16_t &pdts_addr, uint16_t &pdts_ctl);
int pdts_set_addr(uintptr_t &addr,uint16_t pdts_addr);
// alignment laser
int get_align_laser_state(uintptr_t &addr);

// motor
int motor_init_limits();
int set_motor_init_position(motor_t m1, motor_t m2, motor_t m3);
int get_motor_init_position();
int motor_extract_info(const char *arg, int32_t &pos, uint32_t &dir);
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

int get_dna(uintptr_t &addr1,uintptr_t &addr2);

int run_command(int argc, char** argv);
void print_help();
void read_register(mapped_mem_t &reg);


void read_register(mapped_mem_t &reg)
{
  // reads the input register
  spdlog::info("--> Read register on [0x{0:X}]",reg.p_addr);
  uint32_t val_ch0 = cib::util::reg_read(reg.v_addr);
  uint32_t val_ch1 = cib::util::reg_read(reg.v_addr+GPIO_CH_OFFSET);

  spdlog::info("Register values : \n"
    "CH 0 : [{0}] [{0:X}]\n"
    "CH 1 : [{1}] [{1:X}]\n"
      , val_ch0, val_ch1
  );
}


int get_dna(uintptr_t &addr1, uintptr_t &addr2)
{
  // this is a rather special case in which we need to look into two modules to get the whole information
  spdlog::info("Getting the DNA from the silicon");
  uintptr_t maddr = addr1 +(GPIO_CH_OFFSET*0);
  uint32_t word0 = cib::util::reg_read(maddr);
  maddr = addr1 +(GPIO_CH_OFFSET*1);
  uint32_t word1 = cib::util::reg_read(maddr);
  maddr = addr2 +(GPIO_CH_OFFSET*0);
  uint32_t word2 = cib::util::reg_read(maddr);

  spdlog::info("DNA : 0x{2:X} 0x{1:X} 0x{0:X}",word0,word1,word2);
  spdlog::info("DNA (full) : 0x{2:X}{1:X}{0:X}",word0,word1,word2);
  return 0;
}

int lbls_set_width(uintptr_t &addr, uint32_t &width)
{
  int res = 0;
  spdlog::debug("Setting lbls width to ({0}) 0x{0:X}",width);
  uintptr_t maddr = addr +(GPIO_CH_OFFSET*0);
  spdlog::trace("Caching the state register");
  uint32_t state_word = cib::util::reg_read(maddr);
  uint32_t state_mask = cib::util::bitmask(31,31);
  uint32_t state_cache = state_word & state_mask;
  spdlog::trace("Stopping the LBLS triggers (if enabled)");
  (void)lbls_set_state(addr,0x0);
  // writ the new width
  uint32_t mask = cib::util::bitmask(7,0);
  spdlog::trace("Writing 0x{0:X} mask 0x{1:X}",width,mask);
  cib::util::reg_write_mask(maddr,width,mask);
  spdlog::trace("Restoring state");
  cib::util::reg_write_mask(maddr,state_cache,state_mask);

  return res;
}

int lbls_set_state(uintptr_t &addr, uint32_t state)
{
  int res = 0;
  spdlog::debug("Setting the lbls state to  ({0}) 0x{0:X}",state);
  uintptr_t maddr = addr +(GPIO_CH_OFFSET*0);
  uint32_t mask = cib::util::bitmask(31,31);
  cib::util::reg_write_mask_offset(maddr,state,mask,31);
  return res;

}
int lbls_get_status(uintptr_t &addr, uint32_t &state, uint32_t &width)
{
  int res = 0;
  spdlog::debug("Querying LBLS status");
  uintptr_t maddr = addr +(GPIO_CH_OFFSET*0);
  uint32_t reg_val = cib::util::reg_read(maddr);
  uint32_t mask = cib::util::bitmask(31,31);
  state = ((reg_val & mask) >> 31);
  mask = cib::util::bitmask(7,0);
  width = (reg_val & mask);

  return res;
}
int sys_reset(uintptr_t &addr)
{
  spdlog::info("Resetting the system");
  uintptr_t maddr = addr +(GPIO_CH_OFFSET*0);
  uint32_t mask = cib::util::bitmask(27,27);
  cib::util::reg_write_mask_offset(maddr,0x1,mask,27);
  std::this_thread::sleep_for(std::chrono::microseconds(10));
  cib::util::reg_write_mask_offset(maddr,0x0,mask,27);

  return 0;
}

int trigger_pulser_get_state(uintptr_t &addr, uint32_t &state)
{
  spdlog::info("Querying pulser trigger state");
  uintptr_t maddr = addr +(GPIO_CH_OFFSET*0);
  uint32_t mask = cib::util::bitmask(29,29);
  uint32_t reg_val = cib::util::reg_read(maddr);
  state = ((reg_val & mask) >> 29);
  spdlog::info("STATE : {0}",state);
  return 0;
}

int trigger_pulser_set_state(uintptr_t &addr, uint32_t &state)
{
  spdlog::info("Setting pulser trigger state to {0}",state);
  uintptr_t maddr = addr +(GPIO_CH_OFFSET*0);
  uint32_t mask = cib::util::bitmask(29,29);
  cib::util::reg_write_mask_offset(maddr,state,mask,29);
  return 0;
}
int daq_fifo_get_state(uintptr_t &addr, uint32_t &state)
{
  spdlog::info("Querying DAQ queue state");
  uintptr_t maddr = addr +(GPIO_CH_OFFSET*0);
  uint32_t mask = cib::util::bitmask(26,26);
  uint32_t reg_val = cib::util::reg_read(maddr);
  state = ((reg_val & mask) >> 26);
  spdlog::info("STATE : {0}",state);
  return 0;
}

int daq_fifo_set_state(uintptr_t &addr, uint32_t &state)
{
  spdlog::info("Setting DAQ fifo state to {0}",state);
  uintptr_t maddr = addr +(GPIO_CH_OFFSET*0);
  uint32_t mask = cib::util::bitmask(26,26);
  cib::util::reg_write_mask_offset(maddr,state,mask,26);
  return 0;
}

int trigger_ext_get_state(uintptr_t &addr, uint32_t &state)
{
  spdlog::info("Querying ext trigger state");
  uintptr_t maddr = addr +(GPIO_CH_OFFSET*0);
  uint32_t mask = cib::util::bitmask(30,30);
  uint32_t reg_val = cib::util::reg_read(maddr);
  state = ((reg_val & mask) >> 30);
  spdlog::info("STATE : {0}",state);
  return 0;
}
int trigger_ext_set_state(uintptr_t &addr, uint32_t &state)
{
  spdlog::info("Setting ext trigger state to {0}",state);
  uintptr_t maddr = addr +(GPIO_CH_OFFSET*0);
  uint32_t mask = cib::util::bitmask(30,30);
  cib::util::reg_write_mask_offset(maddr,state,mask,30);
  return 0;
}

int shutter_set_state(uintptr_t &addr, uint32_t &state)
{
  spdlog::info("Setting shutter user state to {0}",state);
  uintptr_t maddr = addr +(GPIO_CH_OFFSET*0);
  uint32_t mask = cib::util::bitmask(28,28);
  cib::util::reg_write_mask_offset(maddr,state,mask,28);
  return 0;
}
int shutter_get_state(uintptr_t &addr, uint32_t &state)
{
  spdlog::info("Querying shutter user state");
  uintptr_t maddr = addr +(GPIO_CH_OFFSET*0);
  uint32_t mask = cib::util::bitmask(28,28);
  uint32_t reg_val = cib::util::reg_read(maddr);
  state = ((reg_val & mask) >> 28);
  spdlog::info("STATE : {0}",state);
  return 0;
}


int map_memory()
{

//  spdlog::info("Mapping configuration module");
//  g_mem_fd = 0;
//  g_cib_mem.config.p_addr = CONF_MEM_LOW;
//  g_cib_mem.config.v_addr = cib::util::map_phys_mem(g_mem_fd,CONF_MEM_LOW,CONF_MEM_HIGH);
//  //uintptr_t vmem_conf = cib::util::map_phys_mem(g_mem_fd,CONF_MEM_LOW,CONF_MEM_HIGH);
//  spdlog::debug("\nGot virtual address [0x{:X}]",g_cib_mem.config.v_addr);
//  if (g_cib_mem.config.v_addr == 0x0)
//  {
//    spdlog::critical("Failed to map configuration memory. This is not going to end well.");
//    return 255;
//  }

  spdlog::info("Mapping GPIO_PDTS");
  g_cib_mem.gpio_pdts.p_addr = GPIO_PDTS_MEM_LOW;
  g_cib_mem.gpio_pdts.v_addr = cib::util::map_phys_mem(g_mem_fd,GPIO_PDTS_MEM_LOW,GPIO_PDTS_MEM_HIGH);
  //uintptr_t vmem_pdts = cib::util::map_phys_mem(memfd,GPIO_PDTS_MEM_LOW,GPIO_PDTS_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",g_cib_mem.gpio_pdts.v_addr);
  if (g_cib_mem.gpio_pdts.v_addr == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    return 255;
  }

  spdlog::info("Mapping GPIO_ALIGN");
  g_cib_mem.gpio_align.p_addr = GPIO_ALIGN_MEM_LOW;
  g_cib_mem.gpio_align.v_addr = cib::util::map_phys_mem(g_mem_fd,GPIO_ALIGN_MEM_LOW,GPIO_ALIGN_MEM_HIGH);
//  uintptr_t vmem_align = cib::util::map_phys_mem(memfd,GPIO_ALIGN_MEM_LOW,GPIO_ALIGN_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",g_cib_mem.gpio_align.v_addr);
  if (g_cib_mem.gpio_align.v_addr == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    return 255;
  }

  spdlog::info("Mapping GPIO_LASER");
  g_cib_mem.gpio_laser.p_addr = GPIO_LASER_MEM_LOW;
  g_cib_mem.gpio_laser.v_addr = cib::util::map_phys_mem(g_mem_fd,GPIO_LASER_MEM_LOW,GPIO_LASER_MEM_HIGH);
//  uintptr_t vmem_align = cib::util::map_phys_mem(memfd,GPIO_ALIGN_MEM_LOW,GPIO_ALIGN_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",g_cib_mem.gpio_laser.v_addr);
  if (g_cib_mem.gpio_laser.v_addr == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    return 255;
  }

  spdlog::info("Mapping GPIO_MISC");
  g_cib_mem.gpio_misc.p_addr = GPIO_MISC_MEM_LOW;
  g_cib_mem.gpio_misc.v_addr = cib::util::map_phys_mem(g_mem_fd,GPIO_MISC_MEM_LOW,GPIO_MISC_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",g_cib_mem.gpio_misc.v_addr);
  if (g_cib_mem.gpio_misc.v_addr == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    return 255;
  }

  spdlog::info("Mapping GPIO_I_0");
  g_cib_mem.gpio_mon_0.p_addr = GPIO_I_0_MEM_LOW;

  g_cib_mem.gpio_mon_0.v_addr = cib::util::map_phys_mem(g_mem_fd,GPIO_I_0_MEM_LOW,GPIO_I_0_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",g_cib_mem.gpio_mon_0.v_addr);
  if (g_cib_mem.gpio_mon_0.v_addr == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    return 255;
  }

  spdlog::info("Mapping GPIO_I_1");
  g_cib_mem.gpio_mon_1.p_addr = GPIO_I_1_MEM_LOW;
  g_cib_mem.gpio_mon_1.v_addr = cib::util::map_phys_mem(g_mem_fd,GPIO_I_1_MEM_LOW,GPIO_I_1_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",g_cib_mem.gpio_mon_1.v_addr);
  if (g_cib_mem.gpio_mon_1.v_addr == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    return 255;
  }

  spdlog::info("Mapping MOTOR_1");
  g_cib_mem.gpio_motor_1.p_addr = GPIO_MOTOR_1_MEM_LOW;
  g_cib_mem.gpio_motor_1.v_addr = cib::util::map_phys_mem(g_mem_fd,GPIO_MOTOR_1_MEM_LOW,GPIO_MOTOR_1_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",g_cib_mem.gpio_motor_1.v_addr);
  if (g_cib_mem.gpio_motor_1.v_addr == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    return 255;
  }

  spdlog::info("Mapping MOTOR_2");
  g_cib_mem.gpio_motor_2.p_addr = GPIO_MOTOR_2_MEM_LOW;
  g_cib_mem.gpio_motor_2.v_addr = cib::util::map_phys_mem(g_mem_fd,GPIO_MOTOR_2_MEM_LOW,GPIO_MOTOR_2_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",g_cib_mem.gpio_motor_2.v_addr);
  if (g_cib_mem.gpio_motor_2.v_addr == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    return 255;
  }

  spdlog::info("Mapping MOTOR_3");
  g_cib_mem.gpio_motor_3.p_addr = GPIO_MOTOR_3_MEM_LOW;
  g_cib_mem.gpio_motor_3.v_addr = cib::util::map_phys_mem(g_mem_fd,GPIO_MOTOR_3_MEM_LOW,GPIO_MOTOR_3_MEM_HIGH);
  spdlog::debug("\nGot virtual address [0x{:X}]",g_cib_mem.gpio_motor_3.v_addr);
  if (g_cib_mem.gpio_motor_3.v_addr == 0x0)
  {
    spdlog::critical("Failed to map GPIO memory. Investigate that the address is correct.");
    return 255;
  }

  return 0;
}

void clear_memory()
{
  spdlog::info("Unmapping the register memory");
//  spdlog::debug("* config");
//  int ret = cib::util::unmap_mem(g_cib_mem.config.v_addr,PAGE_SIZE);
//  if (ret != 0)
//  {
//    spdlog::error("Failed to unmap config");
//  }
  int ret = cib::util::unmap_mem(g_cib_mem.gpio_laser.v_addr,PAGE_SIZE);
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
  ret = cib::util::unmap_mem(g_cib_mem.gpio_misc.v_addr,PAGE_SIZE);
  if (ret != 0)
  {
    spdlog::error("Failed to unmap misc");
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
  ret = cib::util::unmap_mem(g_cib_mem.gpio_motor_1.v_addr,PAGE_SIZE);
  if (ret != 0)
  {
    spdlog::error("Failed to unmap motor_1");
  }
  ret = cib::util::unmap_mem(g_cib_mem.gpio_motor_2.v_addr,PAGE_SIZE);
  if (ret != 0)
  {
    spdlog::error("Failed to unmap motor_2");
  }
  ret = cib::util::unmap_mem(g_cib_mem.gpio_motor_3.v_addr,PAGE_SIZE);
  if (ret != 0)
  {
    spdlog::error("Failed to unmap motor_3");
  }
  spdlog::info("Destroying the DAC connection");
  delete g_dac;
  g_dac = nullptr;

  close(g_mem_fd);
 }

int setup_dac(cib::i2c::AD5339 &dac)
{
  spdlog::debug("Configuring the DAC to the appropriate settings [bus = 7; addr = 0xd]");
  int res = dac.set_bus(7);
  if (res != CIB_I2C_OK)
  {
    spdlog::critical("Failed to set bus number. Returned 0x{0:X} ({1})",res,cib::i2c::strerror(res));
    return res;
  }
  res = dac.set_dev_number(0xd);
  if (res != CIB_I2C_OK)
  {
    spdlog::critical("Failed to set dev number. Returned 0x{0:X} ({1})",res,cib::i2c::strerror(res));
    return res;
  }
  res = dac.open_device();
  if (res != CIB_I2C_OK)
  {
    spdlog::critical("Failed to open device. Returned 0x{0:X} ({1})",res,cib::i2c::strerror(res));
    return res;
  }
  return CIB_I2C_OK;
}


// implement soeme extra commands
int set_motor_init_position(motor_t m1, motor_t m2, motor_t m3)
{
  spdlog::info("Setting initial position to [RNN800, RNN600, LSTAGE] = ({0}:{1}, {2}:{3}, {4}:{5})",
               m1.pos_i,m1.dir,m2.pos_i,m2.dir,m3.pos_i,m3.dir);

  if ((m1.pos_i < cib::limits::m_limits.m1_min) || (m1.pos_i > cib::limits::m_limits.m1_max))
  {
    spdlog::error("Requested RNN800/TSTAGE position out of range [{0};{1}]",cib::limits::m_limits.m1_min,cib::limits::m_limits.m1_max);
    return 1;
  }
  if ((m2.pos_i < cib::limits::m_limits.m2_min) || (m2.pos_i > cib::limits::m_limits.m2_max))
  {
    spdlog::error("Requested RNN600 position out of range [{0};{1}]",cib::limits::m_limits.m2_min,cib::limits::m_limits.m2_max);
    return 1;
  }

  if ((m3.pos_i < cib::limits::m_limits.m3_min) || (m3.pos_i > cib::limits::m_limits.m3_max))
  {
    spdlog::error("Requested LSTAGE position out of range [{0};{1}]",cib::limits::m_limits.m3_min,cib::limits::m_limits.m3_max);
    return 1;
  }


  uintptr_t maddr = g_cib_mem.gpio_motor_1.v_addr;
  uint32_t mask = cib::util::bitmask(21,0);
  uint32_t reg = (m1.dir << 31) | cib::util::cast_from_signed(m1.pos_i, mask); // this should be replaced
  spdlog::debug("Writing M1 register with 0x{0}",reg);
  cib::util::reg_write(maddr,reg);
  std::this_thread::sleep_for(std::chrono::microseconds(10));
  maddr = g_cib_mem.gpio_motor_2.v_addr;
  reg = (m2.dir << 31)  | cib::util::cast_from_signed(m2.pos_i, mask); // this should be replaced
  spdlog::debug("Writing M2 register with 0x{0}",reg);
  cib::util::reg_write(maddr,reg);
  std::this_thread::sleep_for(std::chrono::microseconds(10));
  maddr = g_cib_mem.gpio_motor_3.v_addr;
  mask = cib::util::bitmask(16,0);
  reg = (m3.dir << 31)  | cib::util::cast_from_signed(m3.pos_i, mask); // this should be replaced
  spdlog::debug("Writing M3 register with 0x{0}",reg);
  cib::util::reg_write(maddr,reg);
  std::this_thread::sleep_for(std::chrono::microseconds(10));


  // read the register back to be sure
  int32_t m1r,m2r,m3r;
  mask = cib::util::bitmask(21,0);
  m1r = cib::util::cast_to_signed(cib::util::reg_read(g_cib_mem.gpio_motor_1.v_addr), mask);
  m2r = cib::util::cast_to_signed(cib::util::reg_read(g_cib_mem.gpio_motor_2.v_addr), mask);
  mask = cib::util::bitmask(16,0);
  m3r = cib::util::cast_to_signed(cib::util::reg_read(g_cib_mem.gpio_motor_3.v_addr), mask);
  spdlog::info("Position set (readback [{0},{1},{2}])",
               m1r,m2r,m3r);


  return 0;
}


int get_motor_init_position()
{
  uint32_t mask = cib::util::bitmask(21,0);
  uint32_t m1r,m2r,m3r;
  m1r = cib::util::reg_read(g_cib_mem.gpio_motor_1.v_addr);
  m2r = cib::util::reg_read(g_cib_mem.gpio_motor_2.v_addr);
  m3r = cib::util::reg_read(g_cib_mem.gpio_motor_3.v_addr);
  spdlog::info("Raw values: {0:X} {1:X} {2:X}",m1r,m2r,m3r);
  int32_t m1pos = cib::util::cast_to_signed((m1r & mask),mask);
  int32_t m2pos = cib::util::cast_to_signed((m2r & mask),mask);
  mask = cib::util::bitmask(16,0);
  int32_t m3pos = cib::util::cast_to_signed((m3r & mask),mask);

  spdlog::info("Getting initial movement position from motors");
  spdlog::info("Position [RNN800, RNN600, LSTAGE] = [{0},{1},{2}]",m1pos,m2pos,m3pos);

  // now the directions
  mask = cib::util::bitmask(31,31);
  uint32_t m1dir = (m1r & mask) >> 31;
  uint32_t m2dir = (m1r & mask) >> 31;
  uint32_t m3dir = (m1r & mask) >> 31;
  spdlog::trace("Getting movement direction from motors (0 : u, 1: d)");
  spdlog::info("Direction [RNN800, RNN600, LSTAGE] = [{0},{1},{2}]",m1dir,m2dir,m3dir);

  return 0;
}

int motor_init_limits()
{
  // limits from https://docs.google.com/spreadsheets/d/100HDufZ39EIJtkl2HsLmL_xbE8cTSIuBd_5BsyRRMto/edit?usp=sharing
  cib::limits::m_limits.m3_min = -3001;
  cib::limits::m_limits.m3_max = 28967;
  cib::limits::m_limits.m2_min = -580000;
  cib::limits::m_limits.m2_max = 580000;
  cib::limits::m_limits.m1_min = -580000;
  cib::limits::m_limits.m1_max = 580000;

  return 0;
}

int motor_extract_info(const char *arg, int32_t &pos, uint32_t &dir)
{
  // first grab the last char of the arg
  std::string str(arg);
  if (*(str.rbegin()) == 'u')
  {
    dir = 1;
  }
  else if (*(str.rbegin()) == 'd')
  {
    dir = 0;
  }
  else
  {
    spdlog::error("Unknown direction setting [{0}]",str[0]);
    return 1;
  }

  // eliminate the last char
  str.pop_back();

  spdlog::debug("Extracted pos {0} dir {1} (1=u; 0=d)",pos,dir);
  pos = std::strtol(str.c_str(),NULL,0);

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
  spdlog::info("Current settings : width {0} ({1} us) period {2} ({3} us)",width,float(width)*16./1000.0,period,float(period)*16.0/1000.0);
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

  uintptr_t maddr = addr +(GPIO_CH_OFFSET*0);
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
      ,fire_state, fire_width, float(fire_width)*16.0/1000.0,fire_period,float(fire_period)*16.0/1000.0
      ,qs_state, qs_width,float(qs_width)*16.0/1000.0,qs_delay,float(qs_delay)*16.0/1000.0);
  return 0;
}


int set_laser_fire_state(uintptr_t &addr, uint32_t state)
{
  uintptr_t maddr = addr +(GPIO_CH_OFFSET*0);
  uint32_t mask = cib::util::bitmask(31,31);
  cib::util::reg_write_mask_offset(maddr,state,mask,31);
  return 0;
}

int set_laser_qswitch_state(uintptr_t &addr, uint32_t state)
{
  uintptr_t maddr = addr +(GPIO_CH_OFFSET*1);
  uint32_t mask = cib::util::bitmask(31,31);
  cib::util::reg_write_mask_offset(maddr,state,mask,31);
  return 0;
}


int set_laser_fire(uintptr_t &addr, const uint32_t width, const uint32_t period = 6250000)
{
  // first make sure to shut down the laser
  // keep track of the current laser state
  uintptr_t maddr = addr +(GPIO_CH_OFFSET*0);

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
  uintptr_t maddr = addr +(GPIO_CH_OFFSET*0);
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

  // take care of the delay
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
  cib::util::reg_write_mask_offset(memaddr,0x0,mask,31);
  return 0;
}

int pdts_reset_domain(uintptr_t &addr)
{
  spdlog::info("Resetting PDTS domain");
  uintptr_t memaddr = addr+(GPIO_CH_OFFSET*1);
  uint32_t mask = cib::util::bitmask(29,29);
  spdlog::trace("Setting reset high");
  cib::util::reg_write_mask_offset(memaddr,0x1,mask,29);
  spdlog::trace("Sleeping for 10 us");
  std::this_thread::sleep_for(std::chrono::microseconds(10));
  spdlog::trace("Setting reset low");
  cib::util::reg_write_mask_offset(memaddr,0x0,mask,29);
  return 0;
}


int pdts_get_status(uintptr_t &addr, uint16_t &pdts_stat, uint16_t &pdts_addr, uint16_t &pdts_ctl)
{
  // information in the pdts status register:
  // [0:3]  : status
  // [4:11] : ctrl
  // [12:12]  : dna_addr_done
  // [13:15]  : padding
  // [16:31] : address

  // first channel of the GPIO is the read one
  uintptr_t memaddr = addr + (GPIO_CH_OFFSET*0);

  spdlog::debug("Reading memory address 0x{0:X}",memaddr);
  uint32_t reg_val = cib::util::reg_read(addr+(GPIO_CH_OFFSET*0));

  uint32_t mask = cib::util::bitmask(0,3);

  spdlog::debug("Register value 0x{0:X} mask 0x{1:X} status 0x{2:X}",reg_val, mask, (reg_val & mask));
  spdlog::info("PDTS STATUS : 0x{0:X}",(reg_val & mask));
  pdts_stat = (reg_val & mask);

  spdlog::debug("Checking pdts address");
  mask = cib::util::bitmask(16,31);
  spdlog::info("PDTS ADDR : 0x{0:X}",((reg_val & mask)>>16));
  pdts_addr = ((reg_val & mask)>>16);

  spdlog::debug("Checking pdts ctrl register");
  mask = cib::util::bitmask(4,11);
  spdlog::info("PDTS CTRL : 0x{0:X}",((reg_val & mask)>>4));
  pdts_ctl = ((reg_val & mask)>>4);

  spdlog::debug("Checking pdts addr_gen done register");
  mask = cib::util::bitmask(12,12);
  spdlog::info("PDTS ADDR DONE : 0x{0:X}",((reg_val & mask)>>12));
//  pdts_ctl = ((reg_val & mask)>>5);

  return 0;
}

int pdts_set_addr(uintptr_t &addr,uint16_t pdts_addr)
{
  spdlog::debug("Setting pdts address to 0x{0:X}",pdts_addr);

  // first channel of the GPIO is the read one
  uintptr_t memaddr = addr + (GPIO_CH_OFFSET*1);
  spdlog::trace("Setting pdts address in memory address 0x{0:X}",memaddr);
  uint32_t mask = cib::util::bitmask(0,15);
  // need to enable the override
  mask |= cib::util::bitmask(30,30);
  uint32_t reg = 0x0;
  reg |= pdts_addr;
  reg |= cib::util::bitmask(30,30); // enable the user override
  spdlog::debug("pdts register setting value: 0x{0} (mask {1}) ",reg,cib::util::dump_binary(mask));
  cib::util::reg_write_mask(memaddr,reg,mask);

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
      res = get_motor_init_position();
    }
    if ((argc != 4))
    {
      spdlog::warn("usage: motor_init [pi1<dir> pi2<dir> pi3<dir>] (<dir> is 'u' (increasing step) or 'd' (decreasing step)");
      return 0;
    }
    else
    {
      uint32_t d1,d2,d3;
      int32_t p1, p2, p3;

      res = motor_extract_info(argv[1], p1,d1);
      if (res != 0)
      {
        spdlog::error("Failed to RNN800/TSTAGE init settings");
        spdlog::warn("usage: motor_init [pi1<dir> pi2<dir> pi3<dir>] (<dir> is 'u' (increasing step) or 'd' (decreasing step)");
        return 0;
      }
      res = motor_extract_info(argv[2], p2,d2);
      if (res != 0)
      {
        spdlog::error("Failed to get RNN600 init settings");
        spdlog::warn("usage: motor_init [pi1<dir> pi2<dir> pi3<dir>] (<dir> is 'u' (increasing step) or 'd' (decreasing step)");
        return 0;
      }
      res = motor_extract_info(argv[3], p3,d3);
      if (res != 0)
      {
        spdlog::error("Failed to get RNN600 init settings");
        spdlog::warn("usage: motor_init [pi1<dir> pi2<dir> pi3<dir>] (<dir> is 'u' (increasing step) or 'd' (decreasing step)");
        return 0;
      }

      spdlog::debug("Setting motor init position to [RNN800,RNN600,LSTAGE] = [{0},{1},{2}] dir=[{3},{5},{6}]",p1,p2,p3,d1,d2,d3);
      motor_t m1, m2, m3;
      m1.pos_i = p1;
      m2.pos_i = p2;
      m3.pos_i = p3;
      m1.dir = d1;
      m2.dir = d2;
      m3.dir = d3;

      int res = set_motor_init_position(m1,m2,m3);
    }
    if (res != 0)
    {
      spdlog::error("An unknown error was found");
    }
    return 0;
  }
  else if(cmd == "align_config")
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
      spdlog::warn("usage: align_laser width period (in 16 ns units)");
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
      spdlog::warn("usage: align_enable state (1: ON; 0: OFF)");
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
          ,fire_state, fire_width, float(fire_width)*16.0/1000.0
          ,qs_state, qs_width,float(qs_width)*16.0/1000.0,qs_delay,float(qs_delay)*16.0/1000.0);

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
          ,fire_state, fire_width, float(fire_width)*16.0/1000.0,fire_period,float(fire_period)*16.0/1000.0
          ,qs_state, qs_width,float(qs_width)*16.0/1000.0,qs_delay,float(qs_delay)*16.0/1000.0);
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
      spdlog::debug("Getting laser settings");
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
    int res = 0;
    if (argc == 1)
    {
      // just read the settings
      uint32_t state, width;
      res = lbls_get_status(g_cib_mem.gpio_misc.v_addr, state, width);
      if (res == 0)
      {
        spdlog::info("LBLS trigger settings:\n"
            "STATE : {0}\n"
            "WIDTH : {1}\n"
        ,state,width);
      }
    }
    else if (argc == 3)
    {
      if (std::string(argv[1])== "width")
      {
        uint32_t width = std::strtoul(argv[2],NULL,0);
        spdlog::debug("Setting width to {0} (0x{0:X}",width);
        res = lbls_set_width(g_cib_mem.gpio_misc.v_addr,width);
      }
      else if (std::string(argv[1])== "enable")
      {
        uint32_t state = std::strtoul(argv[2],NULL,0);
        spdlog::debug("Setting lbls trigger state to {0}",state);
        res = lbls_set_state(g_cib_mem.gpio_misc.v_addr,state);
      }
      else
      {
        spdlog::warn("Unknown subcommand [{0} {1}]",argv[1],argv[2]);
        spdlog::warn("usage: lbls [width <width> (max 255)]");
        spdlog::warn("usage:      [enable <state> (state: 1 (ON); 0 (OFF))]");
      }
    }
    else
    {
      spdlog::warn("usage: lbls [width <width> (max 255)]");
      spdlog::warn("usage:      [enable <state> (state: 1 (ON); 0 (OFF))]");
    }
    if (res != 0)
    {
      spdlog::error("Failed to execute command");
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
  else if (cmd == "pdts_reset_domain")
  {
    spdlog::info("Resetting the PDTS timing domain");
    int res = pdts_reset_domain(g_cib_mem.gpio_pdts.v_addr);
    if (res != 0)
    {
      spdlog::error("Failed to reset PDTS domain");
    }
    return 0;
  }
  else if (cmd == "sys_reset")
  {
    spdlog::info("Resetting the data queues");
    int res = sys_reset(g_cib_mem.gpio_misc.v_addr);
    if (res != 0)
    {
      spdlog::error("Failed to reset system");
    }
    return 0;
  }
  else if (cmd == "dac")
  {

    if (argc == 1)
    {
      spdlog::info("Querying the DAC level");
      uint16_t level = 0x0;
      int ret = g_dac->get_level(1,level);
      if (ret != CIB_I2C_OK)
      {
        spdlog::error("Failed to get level: [{0} : {1}]",ret,cib::i2c::strerror(ret));
      }
      else
      {
        spdlog::info("DAC level : {0}",level);
      }
    }
    else if (argc == 3)
    {
      if (std::string(argv[1]) == "set")
      {
        uint16_t level = (uint16_t) strtoul(argv[2], NULL, 0);

        if (level > 4095)
        {
          spdlog::warn("Value out of range. Setting to max (0xFFF)");
        }
        spdlog::info("Setting DAC to {0}",level);
        int ret = g_dac->set_level(cib::i2c::AD5339::CH_1,level);
        if (ret != CIB_I2C_OK)
         {
           spdlog::error("Failed to set level: [{0} : {1}]",ret,cib::i2c::strerror(ret));
         }
         else
         {
           spdlog::info("DAC set successfully : {0}",level);
         }
      }
      else
      {
        spdlog::error("Unknown DAC subcommand");
        spdlog::warn("Usage: dac [set <value>]");
        spdlog::warn("Usage: dac [clear]");
      }
    }
    else if (argc == 2)
    {
      if (std::string(argv[1]) == "clear")
      {
        spdlog::info("Clearing DAC");
        g_dac->clear(cib::i2c::AD5339::CH_1);
      }
      else
      {
        spdlog::error("Unknown DAC subcommand");
        spdlog::warn("Usage: dac [set <value>]");
        spdlog::warn("Usage: dac [clear]");
      }
    }
    else
    {
      spdlog::warn("Usage: dac set <value>");
    }
    return 0;
  }
  else if (cmd == "pulse_trigger_enable")
  {
    int res = 0;
    if (argc == 1)
    {
      spdlog::info("Checking status of internal 10 Hz pulser trigger");
      uint32_t state;
      res = trigger_pulser_get_state(g_cib_mem.gpio_misc.v_addr,state);
      if (res == 0)
      {
        spdlog::info("STATE : {0}",state);
      }
    }
    else if (argc == 2)
    {
      uint32_t state = std::strtol(argv[1],NULL,0);
      spdlog::debug("Setting state to {0}",state);
      res = trigger_pulser_set_state(g_cib_mem.gpio_misc.v_addr,state);
    }
    else
    {
      spdlog::warn("usage: pulse_trigger_enable [state] (state: 1(ON), 0(OFF)");
    }
    if (res != 0)
    {
      spdlog::error("Failed to execute command");
    }
    return 0;
  }
  else if (cmd == "shutter")
  {
    int res = 0;
    if (argc == 1)
    {
      spdlog::info("Checking status of user forced shutter state");
      uint32_t state;
      res = shutter_get_state(g_cib_mem.gpio_misc.v_addr,state);
      if (res == 0)
      {
        spdlog::info("STATE : {0}",state);
      }
    }
    else if (argc == 2)
    {
      uint32_t state = std::strtol(argv[1],NULL,0);
      spdlog::debug("Setting state to {0}",state);
      res = shutter_set_state(g_cib_mem.gpio_misc.v_addr,state);
    }
    else
    {
      spdlog::warn("usage: shutter [state] (state: 1(FORCE CLOSE), 0(RELEASE FORCE)");
    }
    if (res != 0)
    {
      spdlog::error("Failed to execute command");
    }
    return 0;
  }
  else if (cmd == "ext_trigger_enable")
  {
    int res = 0;
    if (argc == 1)
    {
      spdlog::info("Checking status of external input at SI1");
      uint32_t state;
      res = trigger_ext_get_state(g_cib_mem.gpio_misc.v_addr,state);
      if (res == 0)
      {
        spdlog::info("STATE : {0}",state);
      }
    }
    else if (argc == 2)
    {
      uint32_t state = std::strtol(argv[1],NULL,0);
      spdlog::debug("Setting state to {0}",state);
      res = trigger_ext_set_state(g_cib_mem.gpio_misc.v_addr,state);
    }
    else
    {
      spdlog::warn("usage: ext_trigger_enable [state] (state: 1(ON), 0(OFF)");
    }
    if (res != 0)
    {
      spdlog::error("Failed to execute command");
    }
    return 0;
  }
  else if (cmd == "daq_enable")
  {
    int res = 0;
    if (argc == 1)
    {
      spdlog::info("Checking status of DAQ stream");
      uint32_t state;
      res = daq_fifo_get_state(g_cib_mem.gpio_misc.v_addr,state);
      if (res == 0)
      {
        spdlog::info("STATE : {0}",state);
      }
    }
    else if (argc == 2)
    {
      uint32_t state = std::strtol(argv[1],NULL,0);
      spdlog::debug("Setting DAQ stream state to {0}",state);
      res = daq_fifo_set_state(g_cib_mem.gpio_misc.v_addr,state);
    }
    else
    {
      spdlog::warn("usage: daq_enable [state] (state: 1(ON), 0(OFF)");
    }
    if (res != 0)
    {
      spdlog::error("Failed to execute command");
    }
    return 0;
  }
  else if (cmd == "dna")
  {
    spdlog::debug("Getting DNA");
    get_dna(g_cib_mem.gpio_mon_0.v_addr, g_cib_mem.gpio_mon_1.v_addr);
    return 0;
  }
  else if (cmd == "read")
  {
    if (argc != 2)
    {
      spdlog::error("usage: read <reg> (pdts,laser,align,misc,mon_0,mon_1)");
    }
    else
    {
      std::string scmd = argv[1];
      if (scmd == "laser")
      {
        read_register(g_cib_mem.gpio_laser);
      }
      else if (scmd == "align")
      {
        read_register(g_cib_mem.gpio_align);
      }
      else if (scmd == "pdts")
      {
        read_register(g_cib_mem.gpio_pdts);
      }
      else if (scmd == "misc")
      {
        read_register(g_cib_mem.gpio_misc);
      }
      else if (scmd == "mon_0")
      {
        read_register(g_cib_mem.gpio_mon_0);
      }
      else if (scmd == "mon_1")
      {
        read_register(g_cib_mem.gpio_mon_1);
      }
      else if (scmd == "motor_1")
      {
        read_register(g_cib_mem.gpio_motor_1);
      }
      else if (scmd == "motor_2")
      {
        read_register(g_cib_mem.gpio_motor_2);
      }
      else if (scmd == "motor_3")
      {
        read_register(g_cib_mem.gpio_motor_3);
      }
      else
      {
        spdlog::error("Unknown register");
        spdlog::error("usage: read <reg> (pdts,laser,align,misc,mon_0,mon_1)");
      }
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
  // NFB: This command is no longer available for releases above v09-00
  //  spdlog::info("  dna");
  //  spdlog::info("    Obtain the DNA value from the FPGA silicon.");
  spdlog::info("  dac [subcmd [subcmd_args]]");
  spdlog::info("    Operates the DAC. Available subcommands:");
  spdlog::info("      set <dac_level> (value between 0 and 4095)");
  spdlog::info("      clear");
  spdlog::info("  ext_trigger_enable [state]");
  spdlog::info("    Enable external trigger connected on the SI1");
  spdlog::info("  pulse_trigger_enable [state]");
  spdlog::info("    Enable pulser 10 Hz pulser trigger");
  spdlog::info("  daq_enable [state]");
  spdlog::info("    Enable DAQ trigger stream");
  spdlog::info("  sys_reset");
  spdlog::info("    Resets everything that is not on the PDTS system");
  spdlog::info("  pdts_reset");
  spdlog::info("    Resets the PDTS system");
  spdlog::info("  pdts_reset_domain");
  spdlog::info("    Resets the PDTS timing domain IPs (but not the system)");
  spdlog::info("  pdts [addr <addr>]");
  spdlog::info("    Gets the current state of the PDTS system");
  spdlog::info("  lbls [state <state>] [width <width>]");
  spdlog::info("    Configure the LBLS trigger. Only one subcommand can be issued at a time");
  spdlog::info("  align_config [width period]");
  spdlog::info("    Config alignment laser");
  spdlog::info("  align_enable [state]");
  spdlog::info("    Enable/disable alignment laser");
  spdlog::info("  motor_init [pi_1 pi_2 pi_3]");
  spdlog::info("    Config the initial position of the motor (in the FPGA)");
  spdlog::info("  laser_config [laser_config fire_state fire_width [fire_period] qs_state qs_width qs_delay]");
  spdlog::info("    Configure the laser system in a single command. WARNING: Careful setting the fire_period");
  spdlog::info("  fire_enable [state]");
  spdlog::info("    Enable/disable laser FIRE ");
  spdlog::info("  fire_config [width [period]]");
  spdlog::info("    Configures the FIRE part of the laser");
  spdlog::info("    WARNING: Do not edit the period, unless you know what you are doing");
  spdlog::info("  qswitch_enable [state]");
  spdlog::info("    Enable/disable laser QSWITCH ");
  spdlog::info("  qswitch_config [width delay]");
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
  spdlog::set_level(spdlog::level::info); // Set global log level to info
  motor_init_limits();

  int c;
  opterr = 0;
  int report_level = SPDLOG_LEVEL_INFO;
  while ((c = getopt (argc, argv, "v")) != -1)
  {
    switch (c)
      {
      case 'v':
        if (report_level > 0)
        {
          report_level--;
        }
        break;
      default: /* ? */
        spdlog::warn("Usage: cib_manager [-v]  (repeated flags further increase verbosity)");
        return 1;
      }
  }
  if (report_level != SPDLOG_LEVEL_INFO)
  {
    spdlog::set_level(static_cast<spdlog::level::level_enum>(report_level)); // Set global log level to info
  }

  spdlog::info("Log level: {0} : {1}",static_cast<int>(spdlog::get_level()),spdlog::level::to_string_view(spdlog::get_level()).data());
  spdlog::trace("Just testing a trace");
  spdlog::debug("Just testing a debug");

  int ret = map_memory();

  if (ret != 0)
  {
    spdlog::critical("Failed to map memory. Clearing out.");
    clear_memory();
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
  //cib::i2c::AD5339 dac;
  g_dac = new cib::i2c::AD5339();
  ret = setup_dac(*g_dac);
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
        return 0;
      }
      if (ret != 0)
      {
        clear_memory();
        return ret;
      }
    }
    else
    {
      clear_memory();
      return 0;
    }
    free(buf);
  }
  clear_memory();
  return 0;
}
