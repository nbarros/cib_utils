#include <iostream>
#include <cstdio>

uint32_t bitmask(uint32_t highbit, uint32_t lowbit)
{
  // sort the bit order or this fails miserably
  if (highbit < lowbit)
  {
    uint32_t tmp = lowbit;
    lowbit = highbit;
    highbit = tmp;
  }

  uint32_t i = ~0U;
    return ~(i << highbit << 1) & (i << lowbit);
}

int main(int argc, char** argv)
{

  printf("Testing bitmask [0:3]\n");
  uint32_t mask = bitmask(0,3);
  printf("Got 0x%X (expected 0xF)\n",mask);

  printf("Testing bitmask [3:0]\n");
  mask = bitmask(3,0);
  printf("Got 0x%X (expected 0xF)\n",mask);

  printf("Testing bitmask [4:4]\n");
  mask = bitmask(4,4);
  printf("Got 0x%X (expected 0x10)\n",mask);

  printf("Testing bitmask [31:31]\n");
  mask = bitmask(31,31);
  printf("Got 0x%X (expected 0x80000000)\n",mask);


  return 0;
}