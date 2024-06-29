/**
 * @file cib_monitor.cxx
 *
 * Developer(s) of this DAQ application have yet to replace this line with a brief description of the application.
 *
 * This is part of the DUNE DAQ Application Framework, copyright 2020.
 * Licensing/copyright details are in the COPYING file that you should have
 * received with this code.
 */

#include <thread>
#include <cstdio>
#include <chrono>
#include <cstdint>
#include <atomic>
#include <cstring>
#include <csignal>
#include <Handler.h>

#include <spdlog/spdlog.h>
#include <spdlog/cfg/env.h>
#include <cib_data_fmt.h>

#ifdef SPDLOG_ACTIVE_LEVEL
#undef SPDLOG_ACTIVE_LEVEL
#endif
//#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO

// the trigger generator pushes a piece of data every 100 ms

volatile std::atomic<bool> g_keep_running;

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

int main(int argc, char* argv[])
{
  // FIXME: Add a simulated mode

  // register the signal handlers
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  spdlog::set_pattern("[%s:%!:%#][%^%L%$] [thread %t] %v");

  spdlog::set_level(spdlog::level::info); // Set global log level to debug
  // if the SPDLOG_LEVEL variable is set, it overrides
  //SPDLOG_LEVEL=info,mylogger=trace

  spdlog::cfg::load_env_levels();
  // if the level was not

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
  SPDLOG_INFO( "spdlog active level {}",SPDLOG_ACTIVE_LEVEL);

//  SPDLOG_TRACE( "trace message %i",12);
//  SPDLOG_DEBUG("debug message");
//  SPDLOG_INFO( "info message");
//  SPDLOG_WARN( "warning message");
//  SPDLOG_ERROR( "error message");
  //SPDLOG_CRITICAL( "This is going to crash down");

  g_keep_running.store(true);

  spdlog::info("Initiating listener service");
  cib::Handler handler_(false);
  handler_.init_listener();

  while(g_keep_running.load())
  {
    // do nothing. Just wait
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  SPDLOG_WARN("Received a termination request. Shutting down the CIB DAQ interface.");
  handler_.stop_listener();
  SPDLOG_DEBUG("Listener should be stopped now. Destroying object.");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  SPDLOG_INFO("All done. Have a nice day!");
  return 0;
}
