/*
 * check_cdr_chip.cpp
 *
 *  Created on: Apr 21, 2024
 *      Author: Nuno Barros
 */




#include <cstdio>
#include <cstdlib>

#include <ADN2814.h>
#include <ADN2814_ratemap.h>

extern "C"
{
#include <unistd.h>
#include <getopt.h>
#include <readline/readline.h>
#include <readline/history.h>

}

#include <spdlog/spdlog.h>

using cib::i2c::ADN2814;


int check_result(int res)
{
  if (res != CIB_I2C_OK)
  {
    spdlog::critical("Failed to execute function. Returned 0x{:X} ({})",res,cib::i2c::strerror(res));
    return 1;
  }
  return 0;
}

void print_usage(const char* prog)
{
  spdlog::warn("Usage : {0} [-b bus] [-d device] [cmd]\n",prog);
}

void print_help()
{
  printf("\n\n=============================================================================================\n");
  printf("Available commands:\n");
  printf("  reset\n");
  printf("    Reset the CDR chip\n");
  printf("  reset_lol_static\n");
  printf("    Reset the LOL static status (misc_4)\n");
  printf("  rate\n");
  printf("    Return the coarse rate reading (both code and corresponding estimate)\n");
  printf("  get_los\n");
  printf("    Return the LOS status [1: LOS; 2: no LOS]\n");
  printf("  get_lol\n");
  printf("    Return the LOL status [1: LOL; 0: no LOL]\n");
  printf("  get_static_lol\n");
  printf("    Get the static LOL status [1: Static LOL until reset (reset_misc_4); 0: waiting for next LOL]\n");
  printf("  set_lol_operation [setting]\n");
  printf("    Set the LOL operation [1: static; 0:normal]\n");
  printf("  set_output_boost [setting]\n");
  printf("    Set the boost output [1: boost output; 0:normal output]\n");
  printf("  set_squelch_mode [setting]\n");
  printf("    Set the squelch mode [1: clk or data; 0:clk and data]\n");
  printf("  set_los_polarity [setting]\n");
  printf("    Set the polarity of LOS [1: active high; 0: active low]\n");
  printf("  exit\n");
  printf("    Closes the command interface\n");
  printf("=============================================================================================\n\n");

}

int print_bit_result(const int ret, const uint32_t val)
{
  if (ret != CIB_I2C_OK)
  {
    spdlog::error("Failed to read the CDR chip. Returned code 0x{0:X} {1}",ret,cib::i2c::strerror(ret));
    return 1;
  }
  else
  {
    spdlog::trace("State : {0} ",val);
    return 0;
  }
}

int run_command(ADN2814 &cdr, int argc,char**argv)
{
  if (argc < 1) return 1;
  std::string cmd(argv[0]); // extract the command
  if (cmd == "exit")
  {
    return 255;
  }
  else if (cmd == "reset")
  {
    spdlog::info("Resetting CDR chip");
    cdr.reset();
    return 0;
  }
  else if (cmd == "reset_misc_4")
  {
    spdlog::info("Resetting CDR MISC[4] bit (static LOL)");
    cdr.reset_misc_4();
    return 0;
  }
  else if (cmd == "rate")
  {
    uint16_t rate;
    int res = cdr.get_data_rate(rate);
    if (res != CIB_I2C_OK)
    {
      spdlog::critical("Failed to read the CDR chip. Returned code 0x{0:X} ({1})",res,cib::i2c::strerror(res));
      return 1;
    }
    else
    {
      spdlog::info("Rate measurement : {0} : {1} MHz",rate,ratemap.at(rate));
      return 0;
    }
  }
  else if (cmd == "get_los")
  {
    uint16_t val;
    int ret = cdr.get_los_status(val);
    return print_bit_result(ret,val);
  }
  else if (cmd == "get_lol")
  {
    uint16_t val;
    int ret = cdr.get_lol_status(val);
    return print_bit_result(ret,val);
  }
  else if (cmd == "get_static_lol")
  {
    uint16_t val;
    int ret = cdr.get_static_lol(val);
    return print_bit_result(ret,val);
  }
  else if (cmd == "set_lol_operation")
  {
    if (argc != 2)
    {
      spdlog::warn("Usage: set_lol_operation operation <1:static; 0: normal>");
      return 0;
    }
    uint16_t val = (uint16_t) strtoul(argv[1], NULL, 0);;
    ADN2814::LOLOP op = static_cast<ADN2814::LOLOP>(val);
    cdr.set_lol_operation(op);
    return 0;
  }
  else if (cmd == "set_output_boost")
  {
    if (argc != 2)
    {
      spdlog::warn("Usage: set_output_boost operation <1:boost; 0: normal>");
      return 0;
    }
    uint16_t val = (uint16_t) strtoul(argv[1], NULL, 0);
    cdr.set_output_boost(!(val == 0));
    return 0;
  }
  else if (cmd == "set_squelch_mode")
  {
    if (argc != 2)
    {
      spdlog::warn("Usage: set_squelch_mode mode <1:OR; 0: AND>");
      return 0;
    }
    uint16_t val = (uint16_t) strtoul(argv[1], NULL, 0);;
    ADN2814::SQUELCH mode = static_cast<ADN2814::SQUELCH>(val);
    cdr.set_squelch_mode(mode);
    return 0;
  }
  else if (cmd == "set_los_polarity")
  {
    if (argc != 2)
    {
      spdlog::warn("Usage: set_los_polarity mode <1:active high; 0: active low>");
      return 0;
    }
    uint16_t val = (uint16_t) strtoul(argv[1], NULL, 0);;
    ADN2814::LOSPOL mode = static_cast<ADN2814::LOSPOL>(val);
    cdr.set_config_los(mode);
    return 0;
  }
  else if (cmd == "help") {
    print_help();
  }
  else
  {
    spdlog::error("Unrecognized Command: {0}",argv[0]);
    return 0;
  }
  return 0;
}

int main( int argc, char**argv)
{
  int c;
  spdlog::set_pattern("[%^%L%$] %v");
  spdlog::set_level(spdlog::level::trace); // Set global log level to debug
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




  ADN2814 cdr;

  spdlog::info("Looking for CDR at address {0}:{1:x}",bus,dev);
  if (check_result(cdr.set_bus(bus)))
  {
    return 1;
  }
  spdlog::trace("Set bus to {0}",bus);
  if (check_result(cdr.set_dev_number(dev)))
  {
    return 1;
  }
  spdlog::trace("Set dev to {0:x}",dev);

  if (check_result(cdr.open_device()))
  {
    return 1;
  }
  spdlog::debug("Device open.");
  if (optind < argc)
  {
    return run_command(cdr,argc-optind,argv+optind);
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
            int ret = run_command(cdr,i,cmd);
            delete [] cmd;
            if (ret == 255) return 0;
            if (ret != 0) return ret;
        } else {
            return 0;
        }
        free(buf);
    }
  }
  spdlog::info("Closing the device");
  cdr.close_device();
  return 0;
}
