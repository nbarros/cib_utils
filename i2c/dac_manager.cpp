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

using cib::i2c::AD5339;


int check_result(int res)
{
  if (res != CIB_I2C_OK)
  {
    printf("Failed to execute function. Returned 0x%X (%s)\n",res,cib::i2c::strerror(res));
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
    printf("Failed to read the CDR chip. Returned code 0x%X (%s)\n",ret,cib::i2c::strerror(ret));
    return 1;
  }
  else
  {
    printf("State : %u \n",val);
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
      printf("Clearning all channelsn");
    }
    else
    {
      ch = static_cast<AD5339::Channel>((uint16_t) strtoul(argv[1], NULL, 0));
    }

    dac.clear(ch);
    return 0;
  }
  else if (cmd == "get_dac")
  {
    if (argc != 2)
    {
      printf("Usage: get_dac <channel>\n");
      return 0;
    }
    uint16_t ch = (uint16_t) strtoul(argv[1], NULL, 0);
    uint16_t level = 0x0;
    int ret = dac.get_level(ch,level);
    if (ret != CIB_I2C_OK)
    {
      printf("Failed to get level: [%d : %s]\n",ret,cib::i2c::strerror(ret));
    }
    else
    {
      printf("DAC level : %hu\n",level);
    }
    return 0;
  }
  else if (cmd == "set_dac")
  {
    if (argc != 3)
    {
      printf("Usage: set_dac <channel> <value>\n");
      return 0;
    }
    uint16_t ch = (uint16_t) strtoul(argv[1], NULL, 0);
    if (ch > 3)
    {
      printf("Invalid channel. Channel must be one of 1, 2 or 3 (for both DACs)\n");
      return 0;
    }
    uint16_t level = (uint16_t) strtoul(argv[2], NULL, 0);
    if (level > 0xFFF)
    {
      printf("Invalid value. Max DAC value is %hu\n",0xFFF);
      return 0;
    }
    // all good. Go for it
    int ret = dac.set_level(ch,level);
    if (ret != CIB_I2C_OK)
    {
      printf("Failed to set level: [%d : %s]\n",ret,cib::i2c::strerror(ret));
    }
    else
    {
      printf("DAC set successfully : %hu\n",level);
    }
    return 0;
  }
  else if (cmd == "help") {
    print_help();
  }
  else
  {
    printf("Unrecognized Command: %s\n",argv[0]);
    return 0;
  }
  return 0;
}

int main( int argc, char**argv)
{
  int c;

  int bus = 7, dev = 0x60;
  opterr = 0;

  while ((c = getopt (argc, argv, "b:d:c:")) != -1)
  {
    switch (c)
      {
      case 'a':
        bus = atoi(optarg);
        break;
      case 'b':
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

  printf("Looking for DAC at address [%d:%X]\n",bus,dev);
  if (check_result(dac.set_bus(bus)))
  {
    return 1;
  }
  if (check_result(dac.set_dev_number(dev)))
  {
    return 1;
  }

  if (check_result(dac.open_device()))
  {
    return 1;
  }

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
