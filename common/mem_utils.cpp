/*
 * mem_utils.cpp
 *
 *  Created on: Mar 15, 2024
 *      Author: Nuno Barros
 */

#include <mem_utils.h>
#include <cstdint>
#include <cstdio>
#include <cstddef>
#include <thread>
#include <chrono>
#include <sys/mman.h>
#include <bitset>
#include <spdlog/spdlog.h>

extern "C"
{
#include <fcntl.h>
#include <inttypes.h>
#include <unistd.h>
};

namespace cib
{
  namespace util
  {

    uintptr_t map_phys_mem(int &fd, uintptr_t base_addr,uintptr_t high_addr)
    {
      void * mapped_addr = nullptr;
      off_t dev_base = base_addr;
      if (fd == 0)
      {
        fd = open("/dev/mem",O_RDWR | O_SYNC);
        if (fd == -1)
        {
          printf("Failed to open /dev/mem");
          fd = 0;
          return 0;
        }
      }
      // Map into user space the area of memory containing the device
      //mapped_addr = mmap(0, 0xFFF(high_addr-base_addr), PROT_READ | PROT_WRITE, MAP_SHARED, fd, dev_base & ~(high_addr-base_addr-1));

      uintptr_t page_addr = (base_addr & (~(getpagesize()-1)));
      uintptr_t page_offset = base_addr - page_addr;       //should be 0

      mapped_addr = mmap(0, getpagesize(), PROT_READ | PROT_WRITE, MAP_SHARED, fd, page_addr);
      if ( (intptr_t)mapped_addr == -1) {
        printf("Failed to map register [0x%" PRIx64 " 0x%" PRIx64 "] into virtual address.",base_addr,high_addr);
        mapped_addr = 0;
      }
      return reinterpret_cast<uintptr_t>(mapped_addr);
    }

    int unmap_mem(void* virt_addr, size_t size)
    {
      if (virt_addr == nullptr)
      {
        return 0;
      }
      return munmap(virt_addr,size);
    }

    int unmap_mem(uintptr_t virt_addr, size_t size)
    {
      return unmap_mem(cast_to_void(virt_addr),size);
    }

    uint32_t reg_read(uintptr_t addr)
    {
      spdlog::trace("Reading register 0x{0:X}",addr);
      std::this_thread::sleep_for(std::chrono::microseconds(10));
      return *(volatile uintptr_t*) addr;
      //*static_cast<volatile uint32_t*>(cast_to_void())
    }

    void reg_write(uintptr_t addr, uint32_t value)
    {
      spdlog::trace("reg_write: Writing 0x{1:X} register 0x{0:X}",addr,value);
      std::this_thread::sleep_for(std::chrono::microseconds(10));
      *(volatile uint32_t*) addr = value;
    }

    // take an address and cast it to void*
    // the interesting part is that we need to make sure that
    void* cast_to_void(uintptr_t val)
    {
      return reinterpret_cast<void *>(val);
    }

//    uint32_t cast_to_uint(void* val)
//    {
//      return reinterpret_cast<uint32_t>(val);
//    }

    uintptr_t cast_to_uintptr(void* val)
    {
      return reinterpret_cast<uintptr_t>(val);
    }

    void reg_write_mask(uintptr_t addr, uint32_t value, uint32_t mask)
    {
      uint32_t cache = reg_read(addr);
      spdlog::trace("reg_write_mask: Cache reg value 0x{0:X} (0x{1:X} | 0x{2:X})",addr,value,mask);
      spdlog::trace("reg_write_mask: Writing to register 0x{0:X} (v: 0x{1:X} | m: 0x{2:X})",addr,value,mask);
      reg_write(addr,((cache & ~mask) | (value & mask)));
    }

    void reg_write_mask_offset(uintptr_t addr, uint32_t value, uint32_t mask, uint32_t offset)
    {
      uint32_t cache = reg_read(addr);
      // now shift the value into the relevant bits
      uint32_t v = (value << offset);
      spdlog::trace("reg_write_mask_offset: Writing to register 0x{0:X} (0x{1:X} | 0x{2:X})",addr,v,mask);
      reg_write(addr,((cache & ~mask) | (v & mask)));
    }

    // from https://stackoverflow.com/questions/35109714/can-someone-explain-how-this-bitmask-code-works
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
    /**
     * Examples:
     * bitSpec(0, 32) -> 0xFFFFFFFF.
     * bitSpec(0, 1) -> 0x00000001.
     * bitSpec(16, 16) -> 0xFFFF0000.
     * bitSpec(0, 16) -> 0x0000FFFF.
     */
    uint32_t bitspec(uint16_t start, uint16_t len)
    {
      return (~0U >> (32 - len)) << start;
    }

    /**
     * generate a string equivalent of a value
     * https://bloomfield.online/posts/convert-a-number-to-a-binary-string-and-back-in-cpp/#:~:text=Number%20to%20binary%20string,as%20an%208%2Dbit%20value.&text=std%3A%3Astring%20str%20%3D%20std,to_string()%3B
     */
    const std::string dump_binary(uint32_t val)
    {
      return std::bitset<32>(val).to_string();
    }

    // converts a masked unsigned value into a signed
    // the mask is always assumed to start at 0, so the value has to be shifted right until the lsb aligns with 0
    int32_t cast_to_signed(uint32_t reg, uint32_t mask)
    {
      // first find the msb in the mask. That will be the signed bit
      uint32_t msb = 0;
      int32_t res = 0;
      for (size_t bit = 31; bit > 0; bit--)
      {
        if ((1U << bit) & mask)
        {
          msb = bit;
          break;
        }
      }
      //spdlog::trace("MSB of the mask is {0}",msb);
      if ((1U<< msb) & reg)
      {
        //spdlog::trace("MSB of the mask is {0}",msb);

        res = bitmask(31, msb+1); // set all bits to 1 above the mask
        res = res | (reg & mask);
        // it is a negative value. Set the msb in the result
      }
      else
      {
        // it is a positive value. No need to set the sign bit
        res = reg;
      }
      return res;
    }

    uint32_t cast_from_signed(int32_t val, uint32_t mask)
    {

      // first find the msb in the mask. That will be the signed bit
      uint32_t msb = 0;
      uint32_t lsb = 0;
      uint32_t res = 0;
      for (size_t bit = 31; bit > 0; bit--)
      {
        if ((1U << bit) & mask)
        {
          msb = bit;
          break;
        }
      }
//      for (size_t bit = 0; bit < 31; bit++)
//      {
//        if ((1U << bit) & mask)
//        {
//          lsb = bit;
//          break;
//        }
//      }

      /*
       * https://stackoverflow.com/questions/25754082/how-to-take-twos-complement-of-a-byte-in-c
       *
        Should be something like
        if (val > 2**(msb-lsb))
          res = val;
          else
          res = val - 2**(msb-lsb+1);

          reinterpret_cast<uint32_t>(res);

      */

      //spdlog::trace("MSB and LSB of the mask are {0} {1}",msb,lsb);

      // now define the max and min values that can be generated

      if (val < 0)
      {
        // in this case the result is already in 2's complement
        res = bitmask(msb,msb);
        // now fill the rest of the area in the bitmask
        res |= (val & bitmask(msb-1,lsb));
      }
      else
      {
        res = (val & bitmask(msb,lsb));
      }
      return res;
    }

  };
};



