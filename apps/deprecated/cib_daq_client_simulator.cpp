/*
 * test_daq_client.cpp
 *
 *  Created on: Mar 24, 2024
 *      Author: Nuno Barros
 */


#include <iostream>
#include <sstream>
#include <cstdio>

// necessary to have getopt
extern "C"
{
#include <unistd.h>
};

#include <readline/readline.h>
#include <readline/history.h>

#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <CIBModuleSim.hpp>
#include <json.hpp>

// logger stuff
// although we could just work with printf's
#include <spdlog/spdlog.h>
#include <spdlog/cfg/env.h>

using json = nlohmann::json;

json get_config()
{
  json cfg;
  cfg["cib"]["sockets"]["receiver"]["host"] = "localhost";
  cfg["cib"]["sockets"]["receiver"]["port"] = 4243;
  cfg["cib"]["sockets"]["receiver"]["timeout"] = 1000;
  cfg["cib"]["misc"]["trigger_stream_enable"]= true;
  cfg["cib"]["misc"]["trigger_stream_output"]= "/home/nbarros/trigger_stream";
  cfg["cib"]["misc"]["trigger_stream_update"]= 1;

  cfg["cib_control_host"] = "localhost";
  cfg["cib_control_port"] = 4242;
  cfg["cib_control_timeout"] = 1000;
  printf("Raw configuration object:\n");
  printf("%s\n",cfg.dump().c_str());

  return cfg;
}


void print_usage(const char *prog) {
    printf( "Usage: %s [-w ip] [cmd] \n", prog);
}

void print_help() {
    printf("Available commands:\n");
    printf("  config\n");
    printf("    Reboot the WIB\n");
    printf("  start_run\n");
    printf("    Start a data taking run\n");
    printf("  stop_run\n");
    printf("    Stop a data taking run\n");
    printf("  help\n");
    printf("    Show this help\n");
    printf("  exit\n");
    printf("    Closes the command interface\n");
}

int run_command(dunedaq::cibmodules::CIBModuleSim &cib, int argc, char **argv)
{
    if (argc < 1) return 1;

    std::string cmd(argv[0]);
    if (cmd == "exit") {
        return 255;
    }
    else if (cmd == "config")
    {
      cib.do_configure(get_config());
    }
    else if (cmd == "start_run")
    {
      json obj;
      cib.do_start(obj);
    }
    else if (cmd == "stop_run")
    {
      json obj;
      cib.do_stop(obj);
    }
    else if (cmd == "help")
    {
      print_help();
    }
    else
    {
      printf("Unrecognized Command: %s\n",argv[0]);
      return 0;
    }
    return 0;
}

int main(int argc, char **argv)
{
    char *ip = (char*)"127.0.0.1";
    spdlog::set_pattern("[%s:%!:%#][%^%L%$] [thread %t] %v");

    spdlog::set_level(spdlog::level::trace); // Set global log level to debug
    // if the SPDLOG_LEVEL variable is set, it overrides
    //SPDLOG_LEVEL=info,mylogger=trace

    spdlog::cfg::load_env_levels();
    // if the level was not

    SPDLOG_INFO( "spdlog active level {}",SPDLOG_ACTIVE_LEVEL);

    signed char opt;
//    while ((opt = getopt(argc, argv, "w:h")) != -1) {
    while ((opt = getopt(argc, argv, "h")) != -1) {
       switch (opt) {
           case 'h':
               print_usage(argv[0]);
               print_help();
               return 1;
//           case 'w':
//               ip = optarg;
//               break;
           default: /* '?' */
               print_usage(argv[0]);
               return 1;
       }
    }

    dunedaq::cibmodules::CIBModuleSim cib_control;


    if (optind < argc) {
        return run_command(cib_control,argc-optind,argv+optind);
    } else {
        char* buf;
        while ((buf = readline(">> ")) != nullptr) {
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
                int ret = run_command(cib_control,i,cmd);
                delete [] cmd;
                if (ret == 255) return 0;
                if (ret != 0) return ret;
            } else {
                return 0;
            }
            free(buf);
        }
    }
    return 0;
}
