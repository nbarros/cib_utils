/*
 * test_lbls_interface.cpp
 *
 *  Created on: May 7, 2024
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
  #include <unistd.h>           // Close() system call
  #include <fcntl.h>              // Flags for open()
  #include <sys/ioctl.h>
  #include <inttypes.h>

}

// reg 0 ised for reset
#define CONF_MEM_LOW   0x00A0040000
#define CONF_MEM_HIGH  0x00A004FFFF


volatile std::atomic<bool> run;

// we want this to be a variation of the read axi fifo with some extra configuration for the lbls system

void lbls_task(int fifo_fd)
{
  spdlog::info("Starting LBLS data streaming thread");
  ssize_t rx_bytes;
  int packets_rx;
  // buffer of received data
  uint8_t buf[256];
  uint64_t ts;

  spdlog::info("Setting up the socket");
  zmq::context_t context;
  zmq::socket_t socket(context, ZMQ_REP);
  spdlog::debug("Setting up zmq client");

  //
  while(run.load())
  {
    // grab something from the FIFO
    rx_bytes = read(fifo_fd,buf,256);
    if (rx_bytes > 0)
    {
      // we have good data, send it
      spdlog::trace("Received {0} bytes",rx_bytes);
      ts = *reinterpret_cast<uint64_t*>(&buf);;
      packets_rx++;
      zmq::message_t msg(sizeof(uint64_t));
      std::memcpy((void*)msg.data(), &buf, sizeof(uint64_t));
      socket.send(msg,zmq::send_flags::none);

      // sleep for a few ms to give time to receive a reply
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      zmq::message_t reply;
      socket.recv(reply,zmq::recv_flags::none);

      std::string reply_str(static_cast<char*>(reply.data()), reply.size());
      spdlog::debug("Received a response with {0} bytes",reply.size());

    }
    // no data is available
  }
  spdlog::info("Left the loop. Thread will terminate");
  // terminate the socket and the context
  socket.close();
  context.close();
}

int configure(uintptr_t &addr)
{
  // configure the CIB with sensible data
  spdlog::info("Configuring the CIB in full blast mode");

  // set up the settings before enabling the modules
  // motors initial position
  int reg = 1;
  const uint32_t reg_size = 0x4; // 4 bytes per register
  // set initial position to 0
  cib::util::reg_write_mask(addr+(reg_size*1),0,((1<<21)-1));
  cib::util::reg_write_mask(addr+(reg_size*2),0,((1<<21)-1));
  cib::util::reg_write_mask(addr+(reg_size*3),0,((1<<21)-1));

  // set fire pulse width to 10 us = 625 clock cycles (62.5 MHz)
  cib::util::reg_write_mask(addr+(reg_size*10),625,((1<<12)-1));
  // set pulse period 100 ms = 6250000 clock cycles (62.5 MHz)
  cib::util::reg_write_mask(addr+(reg_size*12),6250000,((1<<24)-1));
  // set QS pulse width to 10 us = 625 clock cycles (62.5 MHz)
  cib::util::reg_write_mask_offset(addr+(reg_size*10),625,((1<<12)-1) << 12,12);
  // set QS delay width to 160 us = 10000 clock cycles (62.5 MHz)
  cib::util::reg_write_mask(addr+(reg_size*11),10000,((1<<15)-1));

  // set the lbls trigger to width 1
  cib::util::reg_write_mask(addr+(reg_size*14),1,((1<<8)-1));

  // set the calib pulser to 10 Hz
  // set the same properties as the fire signal
  // set pulse period 100 ms = 6250000 clock cycles (62.5 MHz)
  cib::util::reg_write_mask(addr+(reg_size*16),6250000,((1<<24)-1));
  // set fire pulse width to 10 us = 625 clock cycles (62.5 MHz)
  cib::util::reg_write_mask(addr+(reg_size*17),625,((1<<16)-1));

  spdlog::info("Starting to enable stuff");

  spdlog::debug("Enabling the laser fire");
  cib::util::reg_write_mask_offset(addr+(reg_size*0),1,(1<<31),31);

  spdlog::debug("Enabling the laser qswitch");
  cib::util::reg_write_mask_offset(addr+(reg_size*10),1,(1<<31),31);

  spdlog::debug("Enabling the calib laser");
  // bit 26
  cib::util::reg_write_mask_offset(addr+(reg_size*0),1,(1<<26),26);

  // do not start the LBLS right away
  return 0;
}

int main(int argc, char** argv)
{
  run.store(true);
  // initiate spdlog
  spdlog::set_pattern("lbls : [%^%L%$] %v");
  spdlog::set_level(spdlog::level::trace); // Set global log level to debug

  spdlog::trace("Just testing a trace");
  spdlog::debug("Just testing a debug");
  int fifo_fd;

  spdlog::info("Mapping configuration module");
  int memfd = 0;
  uintptr_t vmem_conf = cib::util::map_phys_mem(memfd,CONF_MEM_LOW,CONF_MEM_HIGH);
  spdlog::debug("\nGot address [{:X}]",vmem_conf);
  if (vmem_conf == 0x0)
  {
    spdlog::critical("Failed to map configuration memory. This is not going to end well.");
    return 255;
  }

  // config the stuff with somethign reasonable
  spdlog::info("Configuring the system");
  if (configure(vmem_conf) != 0)
  {
    spdlog::critical("Failed to configure. Bailing out");
    cib::util::unmap_mem(cib::util::cast_to_void(vmem_conf),(CONF_MEM_HIGH-CONF_MEM_LOW));
    return 255;
  }

  spdlog::info("Initiating the readout thread");
  std::thread lbls(lbls_task,fifo_fd);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  spdlog::info("Starting data generation");
  // -- write into the enable_lbls register
  spdlog::debug("Enabling the LBLS trigger");
  // bit 27
  cib::util::reg_write_mask_offset(vmem_conf,1,(1<<27),27);

  std::this_thread::sleep_for(std::chrono::seconds(30));

  cib::util::reg_write_mask_offset(vmem_conf,0,(1<<27),27);

  spdlog::warn("Stopping the thread");
  run.store(false);
  lbls.join();

  return 0;
}

