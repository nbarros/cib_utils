/**
 * @file cib_log_server.cpp
 *
 */

#include <thread>
#include <cstdio>
#include <chrono>
#include <cstdint>
#include <atomic>
#include <cstring>
#include <csignal>
#include <fstream>

#include <spdlog/spdlog.h>
#include <spdlog/cfg/env.h>

#include <cib_mem.h>
#include <cib_data_fmt.h>
#include <mem_utils.h>

#include <axis-fifo.h>

#include <cstdlib>
extern "C"
{
#include <fcntl.h>              // Flags for open()
#include <sys/stat.h>           // Open() system call
#include <sys/types.h>          // Types for open()
#include <unistd.h>             // Close() system call
#include <sys/ioctl.h>

}

#ifdef SPDLOG_ACTIVE_LEVEL
#undef SPDLOG_ACTIVE_LEVEL
#endif
//#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO

// the trigger generator pushes a piece of data every 100 ms

volatile std::atomic<bool> g_keep_running;
volatile std::atomic<bool> g_keep_reading;
int g_dev_fd;
int g_mem_fd;
uintptr_t g_reg_addr;

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
    cib::util::unmap_mem(g_reg_addr,0x1000);
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

void read_task()
{
  spdlog::debug("Initiating read task");
  g_keep_reading.store(true);
  int packets_rx = 0;
  cib::daq::iols_trigger_t word;

  char file_name[200] = "" ;
  time_t rawtime;
  time( & rawtime ) ;
  struct tm local_tm;
  struct tm * timeinfo = localtime_r( & rawtime , &local_tm) ;
  strftime( file_name, sizeof(file_name), "%F_%H.%M.%S.iols.out", timeinfo );
  std::string global_name = "/opt/cib_triggers/cib_data_";
  global_name+= file_name ;


  spdlog::info("Storing current stream into file {0}",global_name);

  // open the output file
  std::ofstream out_file( global_name, std::ios::out | std::ofstream::binary ) ;

  while (g_keep_reading.load())
  {
    packets_rx = read(g_dev_fd, &word, sizeof(word));
    if (packets_rx > 0)
    {
      // received a word
      // -- flush the file each time. It does not cost much and avoids data loss due to caching
      out_file.write(reinterpret_cast<const char*>(&word),sizeof(cib::daq::iols_trigger_t));
      out_file.flush();
      packets_rx++;
    }
    else
    {
      // sleep it over, to reduce on the CPU load
      // on another application, this dropped the CPU usage from 100% to 0.2%
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  spdlog::info("Left the reading loop. Closing the output file {1} with {0} entries",packets_rx,global_name);
  out_file.close();
  spdlog::info("Resetting the FIFO in preparation for future runs");
  int ret = ioctl(g_dev_fd, AXIS_FIFO_RESET_IP);
  if (ret)
  {
    spdlog::error("Failed to issue a reset");
  }
  spdlog::info("Data taking stopped");
}

void  terminate_read_task(std::thread &worker)
{
  g_keep_reading.store(false);
  // wait for a little so that the thread has time to close the file
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  if (worker.joinable())
  {
    spdlog::info("Joining the reading thread");
    worker.join();
  }
}

int main(int argc, char* argv[])
{
  // register the signal handlers
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  spdlog::set_pattern("cib_log : [%s:%!:%#][%^%L%$] [thread %t] %v");

  spdlog::set_level(spdlog::level::info); // Set global log level to debug
  // if the SPDLOG_LEVEL variable is set, it overrides
  //SPDLOG_LEVEL=info,mylogger=trace

  spdlog::cfg::load_env_levels();

  // initiate the globals to sensible values
  g_keep_running = true;
  g_keep_reading = true;
  g_dev_fd = 0;
  g_mem_fd = 0;
  g_reg_addr = 0;


  // parse options
  int c;
  opterr = 0;
  int report_level = SPDLOG_LEVEL_INFO;
  while ((c = getopt (argc, argv, "v")) != -1)
  {
    switch (c)
      {
      case 'h':
        spdlog::warn("Usage: cib_log_server [-v]  (repeated flags further increase verbosity)");
        break;
      case 'v':
        if (report_level > 0)
        {
          report_level--;
        }
        break;
      default: /* ? */
        spdlog::warn("Usage: cib_log_server [-v]  (repeated flags further increase verbosity)");
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
  SPDLOG_INFO( "spdlog active level {0}",SPDLOG_ACTIVE_LEVEL);

//  SPDLOG_TRACE( "trace message %i",12);
//  SPDLOG_DEBUG("debug message");
//  SPDLOG_INFO( "info message");
//  SPDLOG_WARN( "warning message");
//  SPDLOG_ERROR( "error message");
  //SPDLOG_CRITICAL( "This is going to crash down");

  spdlog::info("Initiating listener service in lone ranger mode");

  // this actually works a bit differently
  // the service is always running and periodically checking whether the laser has been turned on.
  // when it has, it initiates a readout and stores the output to a file, until the laser is disabled.
  // therefore, there is no need for a handler, since there are no commands being sent. All it needs is a reader
  // continuously poking the device
  // so we follow a simpler procedure in the lines of the read_daq_fifo

  /***********************************************/
  /* map the laser enable register (for reading) */
  /***********************************************/
  g_reg_addr = cib::util::map_phys_mem(g_mem_fd,GPIO_LASER_MEM_LOW,0x1000);
  if (!g_reg_addr)
  {
    spdlog::critical("Failed to map laser control register");
    cleanup();
    return -1;
  }
  uint32_t reg_stat = cib::util::reg_read(g_reg_addr);

  /*************/
  /* open FIFO */
  /*************/
  g_dev_fd = open(AXIS_FIFO_LOG_DEV, O_RDONLY | O_NONBLOCK);
  if (g_dev_fd < 0)
  {
    spdlog::critical("Open read failed with error: {0}", std::strerror(errno));
    return -1;
  }

  /*****************************/
  /* initialize the fifo core  */
  /*****************************/

  spdlog::trace("Resetting the FIFO");
  int ret = ioctl(g_dev_fd, AXIS_FIFO_RESET_IP);
  if (ret)
  {
    spdlog::error("Failed to issue a reset");
    return -1;
  }

  /*********************/
  /* start the thread  */
  /*********************/
  std::thread worker;
  spdlog::info("Empty thread ID is joinable: {0}",worker.joinable());

  while(g_keep_running.load())
  {
    // check whether the laser is operating
    reg_stat = cib::util::reg_read(g_reg_addr);
    if ((reg_stat & (0x1 << 31)))
    {
      // run is ongoing, lauch the thread, if it is not yet going
      if (worker.joinable())
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
        worker = std::thread(read_task);
      }
    }
    else
    {
      // -- laser is not on
      // if the thread is still running then terminate it
      if (worker.joinable())
      {
        terminate_read_task(worker);
      }
      else
      {
        // nothing going on. Sleep over it.
        // check whether the laser was enabled every 100 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }
  SPDLOG_WARN("Received a service termination request. Shutting down the CIB LOG server.");
  if (worker.joinable())
  {
    spdlog::warn("Received a termination call while there is reading activity. Killing the thread.");
    terminate_read_task(worker);
  }
  cleanup();
  SPDLOG_INFO("All done. Have a nice day!");
  return 0;
}
