#include <iostream>
#include <string>

#include <cinttypes>

using std::string;

enum eeprom {U42=0x50,U41=0x51,U36=0x52,U24=0x54};

typedef struct i2cdev {
  int32_t     fd;
  uint16_t    addr;
  uint16_t    channel;
  std::string devname;
} i2cdev;



