/*
 * mem_utils.h
 *
 *  Created on: Mar 15, 2024
 *      Author: Nuno Barros
 */

#ifndef DEVICE_INCLUDE_MEM_UTILS_H_
#define DEVICE_INCLUDE_MEM_UTILS_H_
#include <cstdint>
#include <cstdio>
#include <string>

using std::size_t;
namespace cib
{
  namespace util
  {
    uintptr_t map_phys_mem(int &fd, uintptr_t base_addr,uintptr_t high_addr);
    int unmap_mem(uintptr_t virt_addr, size_t size);
    int unmap_mem(void* virt_addr, size_t size);
    uint32_t reg_read(uintptr_t addr);
    void reg_write(uintptr_t addr, uint32_t value);
    void reg_write_mask(uintptr_t addr, uint32_t value, uint32_t mask);
    void reg_write_mask_offset(uintptr_t addr, uint32_t value, uint32_t mask, uint32_t offset);

    void* cast_to_void(uintptr_t val);
//    uint32_t cast_to_uint(void* val);
    uintptr_t cast_to_uintptr(void* val);
    uint32_t bitmask(uint32_t lowbit, uint32_t highbit);
    const std::string dump_binary(uint32_t val);

  }
};

#endif /* DEVICE_INCLUDE_MEM_UTILS_H_ */
