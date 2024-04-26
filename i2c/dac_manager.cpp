/*
 * dac_manager.cpp
 *
 *  Created on: Apr 21, 2024
 *      Author: Nuno Barros
 */




#include <cstdio>
#include <cstdlib>

#include <AD5339.h>

extern "C"
{
#include <unistd.h>
#include <getopt.h>
#include <readline/readline.h>
#include <readline/history.h>

}
#include <spdlog/spdlog.h>
//#include <spdlog/cfg/env.h>

using cib::i2c::AD5339;


int check_result(int res)
{
  if (res != CIB_I2C_OK)
  {
    spdlog::critical("Failed to execute function. Returned 0x{:X} ({})\n",res,cib::i2c::strerror(res));
    return 1;
  }
  return 0;
}

void print_usage(const char* prog)
{
  printf("Usage : %s [-b bus] [-d device] [cmd]\n",prog);
}

void print_help()
{
  printf("Available commands (note channel is: 1, 2, 3 (both)):\n");
  printf("  clear [ch]\n");
  printf("    Clear DAC to zeros (if channel is omitted, both channels are cleared)\n");
  printf("  get_dac <ch>\n");
  printf("    Get the current DAC setting of channel <ch>\n");
  printf("  set_dac ch value\n");
  printf("    Set the DAC level of channel <ch> to <value>.If <ch> = 3, both channels are set\n");
  printf("  exit\n");
  printf("    Closes the command interface\n");

}

int print_bit_result(const int ret, const uint32_t val)
{
  if (ret != CIB_I2C_OK)
  {
    spdlog::error("Failed to read the CDR chip. Returned code 0x{%X} (%s)\n",ret,cib::i2c::strerror(ret));
    return 1;
  }
  else
  {
    spdlog::trace("State : %u \n",val);
    return 0;
  }
}

int run_command(AD5339 &dac, int argc,char**argv)
{
  if (argc < 1) return 1;
  AD5339::Channel ch = AD5339::CH_ALL;
  std::string cmd(argv[0]); // extract the command
  if (cmd == "exit")
  {
    return 255;
  }
  else if (cmd == "clear")
  {
    if (argc != 2)
    {
      ch = AD5339::CH_ALL;
      spdlog::debug("Clearing all channels\n");
    }
    else
    {
      ch = static_cast<AD5339::Channel>((uint16_t) strtoul(argv[1], NULL, 0));
    }
    spdlog::trace("Clearing command argument : {0}",static_cast<uint16_t>(ch));
    dac.clear(ch);
    return 0;
  }
  else if (cmd == "get_dac")
  {
    if (argc != 2)
    {
      spdlog::warn("Usage: get_dac <channel>");
      return 0;
    }
    uint16_t ch = (uint16_t) strtoul(argv[1], NULL, 0);
    uint16_t level = 0x0;
    int ret = dac.get_level(ch,level);
    if (ret != CIB_I2C_OK)
    {
      spdlog::error("Failed to get level: [{0} : {}]",ret,cib::i2c::strerror(ret));
    }
    else
    {
      spdlog::info("DAC level : %hu\n",level);
    }
    return 0;
  }
  else if (cmd == "set_dac")
  {
    if (argc != 3)
    {
      spdlog::warn("Usage: set_dac <channel> <value>");
      return 0;
    }
    uint16_t ch = (uint16_t) strtoul(argv[1], NULL, 0);
    if (ch > 3)
    {
      spdlog::error("Invalid channel. Channel must be one of 1, 2 or 3 (for both DACs)");
      return 0;
    }
    uint16_t level = (uint16_t) strtoul(argv[2], NULL, 0);
    if (level > 0xFFF)
    {
      spdlog::error("Invalid value. Max DAC value is {:x}\n",0xFFF);
      return 0;
    }
    // all good. Go for it
    int ret = dac.set_level(ch,level);
    if (ret != CIB_I2C_OK)
    {
      spdlog::error("Failed to set level: [%d : %s]\n",ret,cib::i2c::strerror(ret));
    }
    else
    {
      spdlog::info("DAC set successfully : {}",level);
    }
    return 0;
  }
  else if (cmd == "help") {
    print_help();
  }
  else
  {
    spdlog::error("Unrecognized Command: %s\n",argv[0]);
    return 0;
  }
  return 0;
}

int main( int argc, char**argv)
{
  int c;
  //auto console = spdlog::stdout_color_mt("console");

  //spdlog::get("console")->set_pattern("[%s:%!:%#][%^%L%$] [thread %t] %v");
  spdlog::set_pattern("[%^%L%$] %v");
  spdlog::set_level(spdlog::level::trace); // Set global log level to debug
  // if the SPDLOG_LEVEL variable is set, it overrides
  //SPDLOG_LEVEL=info,mylogger=trace

  //spdlog::cfg::load_env_levels();
  // if the level was not

  spdlog::info( "spdlog active level {}",SPDLOG_ACTIVE_LEVEL);


  int bus = 7, dev = 0x60;
  opterr = 0;


  while ((c = getopt (argc, argv, "b:d:c:")) != -1)
  {
    switch (c)
      {
      case 'b':
        bus = atoi(optarg);
        break;
      case 'd':
        dev = atoi(optarg);
        break;
      case 'h':
        print_usage(argv[0]);
        print_help();
        return 1;
      default: /* ? */
        print_usage(argv[0]);
        return 1;
      }
  }




  AD5339 dac;

  spdlog::info("Looking for DAC at address {0}:{1:x}",bus,dev);

  if (check_result(dac.set_bus(bus)))
  {
    return 1;
  }
  spdlog::trace("Set bus to {0}",bus);
  if (check_result(dac.set_dev_number(dev)))
  {
    return 1;
  }
  spdlog::trace("Set dev to {0:x}",dev);

  if (check_result(dac.open_device()))
  {
    return 1;
  }
  spdlog::debug("Device open.");
  if (optind < argc)
  {
    return run_command(dac,argc-optind,argv+optind);
  }
  else
  {
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
            int ret = run_command(dac,i,cmd);
            delete [] cmd;
            if (ret == 255) return 0;
            if (ret != 0) return ret;
        } else {
            return 0;
        }
        free(buf);
    }
  }

  dac.close_device();
  return 0;
}
