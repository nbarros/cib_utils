/*
 * test_lbls_interface.cpp
 *
 *  Created on: May 7, 2024
 *      Author: Nuno Barros
 */

#include <thread>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <fstream>
#include <string>

// spdlog headers
#include <spdlog/spdlog.h>
#include <spdlog/cfg/env.h>

// ZMQ headers
#include <zmq.hpp>

extern "C"
{
#include <unistd.h>           // Close() system call
#include <fcntl.h>              // Flags for open()
#include <sys/ioctl.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
}

#include <mem_utils.h>
#include <cib_mem.h>
#include <cib_data_fmt.h>
#include <mem_utils.h>
#include <axis-fifo.h>

//#define VIRTUAL_MODE 1
// Note: leave VIRTUAL_MODE selection to build flags; do not force-undef here.

#define LBLS_SRV "10.73.137.151"
#define LBLS_PORT 9001

// Runtime-overridable endpoints (default to macros above)
static std::string  g_lbls_host = LBLS_SRV;
static uint16_t     g_lbls_port = LBLS_PORT;
// CIB Slow Control (OPC UA) server URI; optional override via CLI
static std::string  g_sc_host = "localhost";
static uint16_t     g_sc_port = 9001;

#ifdef SPDLOG_ACTIVE_LEVEL
#undef SPDLOG_ACTIVE_LEVEL
#endif
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
// #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO

// -- Memory mapped information concerning the LBLS
using cib::lbls_ctl_t;

    volatile std::atomic<bool>
        g_keep_running;
volatile std::atomic<bool> g_keep_reading;
int g_dev_fd;
int g_mem_fd;
uintptr_t g_reg_addr;

volatile std::atomic<bool> run;

// we want this to be a variation of the read axi fifo with some extra configuration for the lbls system

void signal_handler(int signal)
{
  // only answer to sigterm
  if (signal == SIGTERM)
  {
    SPDLOG_WARN("Received SIGTERM. Stopping execution");
    g_keep_running.store(false);
  }
  if (signal == SIGINT)
  {
    SPDLOG_WARN("Received SIGINT. Stopping execution");
    g_keep_running.store(false);
  }
}

void cleanup()
{
  spdlog::info("Cleaning up memory allocations and mappings");
  if (g_reg_addr)
  {
    cib::util::unmap_mem(g_reg_addr, 0x1000);
  }
  if (g_mem_fd)
  {
    close(g_mem_fd);
  }
  if (g_dev_fd)
  {
    close(g_dev_fd);
  }
}

int init_mem_maps()
{

  /***********************************************/
  /* map the laser enable register (for reading) */
  /***********************************************/
  g_reg_addr = cib::util::map_phys_mem(g_mem_fd, GPIO_MISC_MEM_LOW, 0x1000);
  if (!g_reg_addr)
  {
    spdlog::critical("Failed to map MISC register");
    cleanup();
    return -1;
  }
  // all good
  return 0;
}

int init_axi_fifo()
{
  //
  // open the AXI FIFO device
  //
  spdlog::info("Opening LBLS data device");
  g_dev_fd = open(AXIS_FIFO_PTR_DEV, O_RDONLY | O_NONBLOCK);
  if (g_dev_fd < 0)
  {
    spdlog::error("Open read failed with error: {0}", std::strerror(errno));
    return 1;
  }
  // initialize the fifo core
  spdlog::trace("Resetting the FIFO");
  int ret = ioctl(g_dev_fd, AXIS_FIFO_RESET_IP);
  if (ret)
  {
    spdlog::error("Failed to issue a reset");
    return 1;
  }
  return 0;
}

void get_log_file_name(char* buffer, size_t buflen)
{
  // generate a log file name based on timestamp
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);
  std::snprintf(buffer, buflen, "cib_lbls_log_%04d%02d%02d_%02d%02d%02d.out",
                tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                tm.tm_hour, tm.tm_min, tm.tm_sec);

  spdlog::debug("Storing LBLS stream into file {0}", buffer);
}

void open_log_file(std::ofstream &out_file)
{
  char log_file_name[200];
  get_log_file_name(reinterpret_cast<char *>(log_file_name), sizeof(log_file_name));
  std::string full_log_name = "/opt/cib_triggers/";
  full_log_name += log_file_name;

  spdlog::info("Opening output file {0}", full_log_name);
  out_file.open(full_log_name, std::ios::out | std::ofstream::binary);
  if (!out_file.is_open())
  {
    spdlog::error("Failed to open output file {}", full_log_name);
  }
}

void lbls_task(int fifo_fd)
{
  spdlog::info("Starting LBLS data streaming thread");
  g_keep_reading.store(true);

  ssize_t bytes_rx = 0;
  ssize_t read_bytes = 0;
  ssize_t packets_tx = 0;
  ssize_t packets_rx = 0;

  // buffer of data from the FPGA
  cib::daq::iols_trigger_t word;

  // buffer of received data
  uint8_t buf[0xFFFF];
  // timestamp
  uint64_t ts = 0;

  int n;
  socklen_t len;
  // binary file with trigger data and LBLS output
  std::ofstream out_file;
  open_log_file(out_file);
  if (!out_file.is_open())
  {
    spdlog::error("Output file is not open. Terminating thread.");
    return;
  }

#ifdef VIRTUAL_MODE
  // UDP socket thingy

  int sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if(sockfd <0 )
  {
    spdlog::error("Invalid socket");
    return;
  }

  sockaddr_in addr;
  addr.sin_addr.s_addr = inet_addr(g_lbls_host.c_str());
  addr.sin_family = AF_INET;
  addr.sin_port = htons(g_lbls_port);

  len = sizeof(addr);

  int result = connect(sockfd, (sockaddr*)&addr, sizeof(addr));
  if(result < 0 )
  {
    spdlog::error("Socket error");
    return;
  }

  // now start working
  while(run.load())
  {
    ts++;
    //result = send(sockfd, (char*)&ts, sizeof(ts), 0);
    result = sendto(sockfd, (const char *)&ts, sizeof(ts),0,(const struct sockaddr *) &addr, len);
    //        MSG_CONFIRM, (const struct sockaddr *) &addr, len);
    if(result < 0)
    {
      spdlog::error("Socket error at packet {}",packets_tx);
      return;
    }
    else
    {
      spdlog::trace("Got result {0}",result);
    }
    packets_tx++;

    /** Alternative code. Not tested!!!
     *
       socklen_t len;

      sendto(sockfd, (const char *)hello, strlen(hello),
        MSG_CONFIRM, (const struct sockaddr *) &addr,
            sizeof(addr));
     *
     */


    /* To receive a reply
     len = sizeof(addr);  //len is value/result

    n = recvfrom(sockfd, (char *)buffer, MAXLINE,
                   MSG_WAITALL, (struct sockaddr *) &servaddr,
                   &len);
       buffer[n] = '\0';
       std::cout<<"Server :"<<buffer<<std::endl;
    rx_bytes += n;
     */
  }
  spdlog::info("Terminating.Closing socket.");
  spdlog::info("Network statistics: packets_tx {0} bytes_tx {1} packets_rx {2} bytes_rx {3}",packets_tx,sizeof(ts)*packets_tx,packets_rx,rx_bytes);

  close(sockfd);



#else
  spdlog::info("Setting up the socket");
  zmq::context_t context;
  zmq::socket_t socket(context, ZMQ_REQ);
  spdlog::debug("Setting up zmq client in request mode");
  std::string zmq_addr = std::string("tcp://") + g_lbls_host + ":" + std::to_string(g_lbls_port);
  spdlog::info("Connecting to {}", zmq_addr);

  socket.connect(zmq_addr);
  spdlog::trace("Connected to {}", zmq_addr);
  //
  spdlog::trace("Entering loop");
  while (g_keep_reading.load())
  {
    //  -- read from the FIFO
    read_bytes = read(g_dev_fd, &word, sizeof(word));
    if (read_bytes > 0)
    {
      // received a word
      spdlog::trace("Got a word. TS {0}", word.timestamp);
      // -- flush the file each time. It does not cost much and avoids data loss due to caching
      out_file.write(reinterpret_cast<const char *>(&word), sizeof(cib::daq::iols_trigger_t));
      out_file.flush();
      packets_rx++;
      bytes_rx += read_bytes;

      // send the timestamp via ZMQ
      // this should be non blocking
      zmq::message_t msg(sizeof(uint64_t));
      std::memcpy((void*)msg.data(), &word.timestamp, sizeof(uint64_t));
      socket.send(msg, zmq::send_flags::none);
      spdlog::trace("Sent TS {0} via ZMQ", word.timestamp);
      // sleep for a few ms to give time to receive a reply
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      // read a reply message. We do know what is its size      
      zmq::message_t reply;
      spdlog::trace("Waiting for reply");
      zmq::recv_result_t result = socket.recv(reply, zmq::recv_flags::none);
      if (!result)
      {
        spdlog::error("Failed to receive reply from LBLS server");
        continue;
      }

      std::string reply_str(static_cast<char*>(reply.data()), reply.size());
      spdlog::debug("Received a response with {0} bytes", reply.size());

      // dump the contents into the log file
      out_file.write(reply_str.data(), reply.size());
      out_file.flush();

    }
    else
    {
      // sleep it over, to reduce on the CPU load
      // on another application, this dropped the CPU usage from 100% to 0.2%
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

    //    zmq::message_t msg(sizeof(uint64_t));
    //    std::memcpy((void*)msg.data(), &buf, sizeof(uint64_t));
    //    socket.send(msg,zmq::send_flags::none);
    //    // sleep for a few ms to give time to receive a reply
    //    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //zmq::message_t reply;
    //socket.recv(reply,zmq::recv_flags::none);

    // grab something from the FIFO
    //    rx_bytes = read(fifo_fd,buf,256);
    //    if (rx_bytes > 0)
    //    {
    //      // we have good data, send it
    //      spdlog::trace("Received {0} bytes",rx_bytes);
    //      ts = *reinterpret_cast<uint64_t*>(&buf);;
  //   ts += 1;
  //   packets_rx++;
  //   zmq::message_t msg(sizeof(uint64_t));
  //   //std::memcpy((void*)msg.data(), &buf, sizeof(uint64_t));
  //   std::memcpy((void*)msg.data(),(const char*)&ts,sizeof(uint64_t));
  //   spdlog::trace("Sending {0}",ts);
  //   socket.send(msg,zmq::send_flags::none);

  //   // sleep for a few ms to give time to receive a reply
  //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
  //   zmq::message_t reply;
  //   spdlog::trace("Waiting for reply");
  //   socket.recv(reply,zmq::recv_flags::none);

  //   std::string reply_str(static_cast<char*>(reply.data()), reply.size());
  //   spdlog::debug("Received a response with {0} bytes",reply.size());
  //   // }
  //   // no data is available
  // }
  spdlog::info("Left the loop. Thread will terminate");
  out_file.close();
  spdlog::info("LBLS statistics: packets_rx {0} bytes_rx {1}", packets_rx, bytes_rx);
  // terminate the socket and the context
  socket.close();
  context.close();
#endif

}

// int configure(uintptr_t &addr)
// {
//   // configure the CIB with sensible data
//   spdlog::info("Configuring the CIB in full blast mode");
//
//   // set up the settings before enabling the modules
//   // motors initial position
//   int reg = 1;
//   const uint32_t reg_size = 0x4; // 4 bytes per register
//   // set initial position to 0
//   spdlog::debug("Setting motor initial position to 0");
//   //  cib::util::reg_write_mask(addr+(reg_size*1),0,((1<<21)-1));
//   //  cib::util::reg_write_mask(addr+(reg_size*2),0,((1<<21)-1));
//   //  cib::util::reg_write_mask(addr+(reg_size*3),0,((1<<21)-1));
//   cib::util::reg_write(addr+(reg_size*1),0);
//   cib::util::reg_write(addr+(reg_size*2),0);
//   cib::util::reg_write(addr+(reg_size*3),0);
//
//   spdlog::debug("Configuring the laser for standard settings");
//   // set fire pulse width to 10 us = 625 clock cycles (62.5 MHz)
//   cib::util::reg_write_mask(addr+(reg_size*10),625,((1<<12)-1));
//   // set pulse period 100 ms = 6250000 clock cycles (62.5 MHz)
//   cib::util::reg_write_mask(addr+(reg_size*12),6250000,((1<<24)-1));
//   // set QS pulse width to 10 us = 625 clock cycles (62.5 MHz)
//   cib::util::reg_write_mask_offset(addr+(reg_size*10),625,((1<<12)-1) << 12,12);
//   // set QS delay width to 160 us = 10000 clock cycles (62.5 MHz)
//   //cib::util::reg_write_mask(addr+(reg_size*11),10000,((1<<15)-1));
//   cib::util::reg_write(addr+(reg_size*11),10000);
//
//   spdlog::debug("Enabling the LBLS trigger width to 1");
//   // set the lbls trigger to width 1
//   cib::util::reg_write_mask(addr+(reg_size*14),2,((1<<8)-1));
//
//   spdlog::debug("Setting the pulser to 10 Hz");
//   // set the calib pulser to 10 Hz
//   // set the same properties as the fire signal
//   // set pulse period 100 ms = 6250000 clock cycles (62.5 MHz)
//   cib::util::reg_write_mask(addr+(reg_size*16),6250000,((1<<24)-1));
//   // set fire pulse width to 10 us = 625 clock cycles (62.5 MHz)
//   //cib::util::reg_write_mask(addr+(reg_size*17),625,((1<<16)-1));
//   cib::util::reg_write(addr+(reg_size*17),625);
//
//   spdlog::info("Starting to enable stuff");
//
//   spdlog::debug("Enabling the laser fire");
//   cib::util::reg_write_mask_offset(addr+(reg_size*0),1,(1<<31),31);
//
//   spdlog::debug("Enabling the laser qswitch");
//   cib::util::reg_write_mask_offset(addr+(reg_size*10),1,(1<<31),31);
//
//   spdlog::debug("Enabling the calib laser");
//   // bit 26
//   cib::util::reg_write_mask_offset(addr+(reg_size*0),1,(1<<26),26);
//
//   // do not start the LBLS right away
//   return 0;
// }

void terminate_lbls_task(std::thread &worker)
{
  spdlog::info("Terminating LBLS data streaming thread");
  g_keep_reading.store(false);
  // wait for a little so that the thread has time to close the file
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  if (worker.joinable())
  {
    spdlog::info("Joining the LBLS thread");
    worker.join();
  }
}

int main(int argc, char** argv)
{
  // register the signal handlers
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  g_keep_running.store(true);
  // initiate spdlog
  spdlog::set_pattern("lbls : [%^%L%$] %v");
  spdlog::set_level(spdlog::level::trace); // Set global log level to debug
  spdlog::cfg::load_env_levels();

  // initiate the globals to sensible values
  g_keep_running = true;
  g_keep_reading = true;
  g_dev_fd = 0;
  g_mem_fd = 0;
  g_reg_addr = 0;
  uint32_t reg_stat = 0;
  // parse options
  int c;
  opterr = 0;
  int report_level = SPDLOG_LEVEL_INFO;
  // -v increases verbosity; -L <host> overrides LBLS host; -P <port> overrides LBLS port; -C <uri> sets Slow Control (OPC UA) URI
  while ((c = getopt(argc, argv, "vL:P:s:p:h")) != -1)
  {
    switch (c)
    {
      case 'v':
        if (report_level > 0) { report_level--; }
        break;
      case 'L':
        if (optarg) { g_lbls_host = optarg; }
        break;
      case 'P':
        if (optarg) { g_lbls_port = static_cast<uint16_t>(std::strtoul(optarg, nullptr, 10)); }
        break;
      case 's':
        if (optarg) { g_sc_host = optarg; }
        break;
      case 'p':
        if (optarg) { g_sc_port = static_cast<uint16_t>(std::strtoul(optarg, nullptr, 10)); }
        break;
      case 'h':
        spdlog::warn("Usage: cib_lbls_server [-v] [-L <lbls_host>] [-P <lbls_port>] [-C <opcua_uri>]");
        return 0;
      default: /* ? */
        spdlog::warn("Usage: cib_lbls_server [-v] [-L <lbls_host>] [-P <lbls_port>] [-C <opcua_uri>]");
        return 1;
    }
  }
  if (report_level != SPDLOG_LEVEL_INFO)
  {
    spdlog::set_level(static_cast<spdlog::level::level_enum>(report_level)); // Set global log level to info
  }

  spdlog::info("LBLS endpoint: {}:{}", g_lbls_host, g_lbls_port);
  spdlog::info("Slow Control endpoint: {}:{}", g_sc_host, g_sc_port);
  spdlog::info("Log level: {0} : {1}", static_cast<int>(spdlog::get_level()), spdlog::level::to_string_view(spdlog::get_level()).data());

  spdlog::trace("Just testing a trace");
  spdlog::debug("Just testing a debug");
  SPDLOG_WARN("spdlog active level {0}", SPDLOG_ACTIVE_LEVEL);

  spdlog::debug("Initializing listener service");
  int res = init_mem_maps();
  if (res != 0)
  {
    spdlog::critical("Failed to initialize the memory maps. Cannot continue.");
    return 255;
  }
  
  //
  // initialize the AXI FIFO for reading
  //

  res = init_axi_fifo();
  if (res != 0)
  {
    spdlog::critical("Failed to initialize the AXI FIFO. Cannot continue.");
    return 255;
  }

  //
  // Start the worker thread logic
  //
  // this must be in a loop to be able to relaunch when necessary
  //
  std::thread worker_thread;
  spdlog::debug("Empty thread ID (before assignment) is joinable: {0} (expect false)", worker_thread.joinable());
  const uint32_t reg_mask = cib::util::bitmask(26, 26);

  while (g_keep_running.load())
  {
    spdlog::info("Starting LBLS data acquisition");
    // check that the laser is operating
    reg_stat = cib::util::reg_read(g_reg_addr);
    if (reg_stat & reg_mask)
    {
      // run is ongoing, lauch the thread, if it is not yet going
      if (worker_thread.joinable())
      {
        // there is a thread ongoing. Do nothing and loop again
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      else
      {
        // there is no thread. Start one.
        // ensure that we are telling it to keep reading
        g_keep_reading.store(true);
        worker_thread = std::thread(lbls_task, g_dev_fd);
      }
    }
    else
    {
      // -- the run has stopped
      // if the thread is still running then terminate it
      if (worker_thread.joinable())
      {
        terminate_lbls_task(worker_thread);
      }
      else
      {
        // nothing going on. Sleep it over.
        // check whether a run started every 100 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  SPDLOG_WARN("Received a service termination request. Shutting down the CIB LBLS server.");

  if (worker_thread.joinable())
  {
    spdlog::warn("Received a termination call while there is reading activity. Killing the thread.");
    terminate_lbls_task(worker_thread);
  }
  cleanup();
  SPDLOG_INFO("All done. Have a nice day!");
  return 0;

}

