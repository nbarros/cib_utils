/*
 * cib_time.cpp
 *
 *  Created on: Oct 23, 2024
 *      Author: Nuno Barros
 */

#include <cib_time.h>
#include <cib_mem.h>
#include <stdexcept>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

namespace cib
{
  namespace util
  {
    // Initialize static member
    cib_time* cib_time::s_instance = nullptr;

    cib_time& cib_time::get()
    {
      if (!s_instance)
      {
        // Lazy initialization with default address from cib_mem.h
        s_instance = new cib_time(GPIO_TSTAMP_MEM_LOW);
      }
      return *s_instance;
    }

    void cib_time::initialize(uintptr_t timestamp_addr)
    {
      if (s_instance)
      {
        // Destroy existing instance if different address requested
        delete s_instance;
      }
      s_instance = new cib_time(timestamp_addr);
    }

    bool cib_time::isInitialized()
    {
      return s_instance != nullptr;
    }

    cib_time::cib_time(uintptr_t timestamp_addr)
      : m_timestamp_addr(timestamp_addr)
      , m_timestamp_ptr(nullptr)
      , m_timestamp_struct()
    {
      // Open /dev/mem for memory-mapped I/O
      int fd = open("/dev/mem", O_RDWR | O_SYNC);
      if (fd < 0)
      {
        throw std::runtime_error("Failed to open /dev/mem for timestamp access");
      }

      // Map the physical address to virtual address
      // Use PAGE_SIZE alignment
      uintptr_t page_base = (m_timestamp_addr / PAGE_SIZE) * PAGE_SIZE;
      uintptr_t page_offset = m_timestamp_addr - page_base;

      void* mapped_base = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, 
                               MAP_SHARED, fd, page_base);
      
      close(fd);

      if (mapped_base == MAP_FAILED)
      {
        throw std::runtime_error("Failed to mmap timestamp register");
      }

      // Calculate the actual pointer with offset
      volatile uint32_t* ptr = reinterpret_cast<volatile uint32_t*>(
        static_cast<uint8_t*>(mapped_base) + page_offset);
      
      m_timestamp_ptr = std::unique_ptr<volatile uint32_t*>(new volatile uint32_t*(ptr));
    }

    cib_time::~cib_time()
    {
      if (m_timestamp_ptr)
      {
        // Unmap the memory
        uintptr_t page_base = (m_timestamp_addr / PAGE_SIZE) * PAGE_SIZE;
        uintptr_t page_offset = m_timestamp_addr - page_base;
        
        void* mapped_base = reinterpret_cast<void*>(
          reinterpret_cast<uintptr_t>(*m_timestamp_ptr.get()) - page_offset);
        
        munmap(mapped_base, PAGE_SIZE);
      }
      
      // Clear the singleton instance pointer
      if (s_instance == this)
      {
        s_instance = nullptr;
      }
    }

    uint64_t cib_time::get_timestamp()
    {
      if (!m_timestamp_ptr)
      {
        throw std::runtime_error("Timestamp pointer not initialized");
      }
      
      // Read from two 32-bit registers
      // Low register at base address, high register at base + 0x8
      volatile uint32_t* base_ptr = *m_timestamp_ptr.get();
      
      // Read low 32 bits from base address
      m_timestamp_struct.low = *base_ptr;
      
      // Read high 32 bits from base + 0x8 (offset of 2 in 32-bit words)
      m_timestamp_struct.high = *(base_ptr + 2);
      
      return m_timestamp_struct.get_timestamp();
    }

    int64_t cib_time::get_seconds_since_epoch(uint64_t raw_timestamp)
    {
      // Convert clock ticks (16 ns periods) to seconds
      // raw_timestamp is in units of 16 ns
      // 1 second = 1,000,000,000 ns / 16 ns = 62,500,000 ticks
      return static_cast<int64_t>(raw_timestamp / get_clock_frequency_hz());
    }

    int64_t cib_time::get_milliseconds_since_epoch(uint64_t raw_timestamp)
    {
      // Convert clock ticks (16 ns periods) to milliseconds
      // 1 ms = 1,000,000 ns / 16 ns = 62,500 ticks
      const uint64_t ticks_per_ms = get_clock_frequency_hz() / 1000;
      return static_cast<int64_t>(raw_timestamp / ticks_per_ms);
    }

    std::string cib_time::format_timestamp(uint64_t raw_timestamp, 
                                           const std::string& format_string)
    {
      // Convert to seconds since epoch
      time_t seconds = static_cast<time_t>(get_seconds_since_epoch(raw_timestamp));
      
      // Convert to struct tm
      struct tm timeinfo;
      if (gmtime_r(&seconds, &timeinfo) == nullptr)
      {
        throw std::runtime_error("Failed to convert timestamp to tm structure");
      }
      
      // Format the time string
      char buffer[256];
      size_t result = strftime(buffer, sizeof(buffer), format_string.c_str(), &timeinfo);
      
      if (result == 0)
      {
        throw std::runtime_error("Failed to format timestamp string");
      }
      
      return std::string(buffer);
    }

    uint64_t cib_time::calc_timestamp(const std::string& ts, 
                                      const std::string& format_string)
    {
      struct tm timeinfo;
      std::memset(&timeinfo, 0, sizeof(timeinfo));
      
      // Parse the time string
      char* result = strptime(ts.c_str(), format_string.c_str(), &timeinfo);
      
      if (result == nullptr)
      {
        throw std::runtime_error("Failed to parse timestamp string: " + ts);
      }
      
      // Convert to time_t (seconds since epoch)
      time_t seconds = timegm(&timeinfo);
      
      if (seconds == -1)
      {
        throw std::runtime_error("Failed to convert tm structure to timestamp");
      }
      
      // Convert seconds to clock ticks (16 ns periods)
      uint64_t raw_timestamp = static_cast<uint64_t>(seconds) * get_clock_frequency_hz();
      
      return raw_timestamp;
    }

    double cib_time::convert_bits_to_float(uint64_t bits, bool is_double_precision)
    {
      if (is_double_precision)
      {
        // Interpret as 64-bit double
        double result;
        std::memcpy(&result, &bits, sizeof(double));
        return result;
      }
      else
      {
        // Interpret as 32-bit float (use lower 32 bits)
        uint32_t bits32 = static_cast<uint32_t>(bits & 0xFFFFFFFF);
        float result;
        std::memcpy(&result, &bits32, sizeof(float));
        return static_cast<double>(result);
      }
    }

    uint64_t cib_time::to_ua_datetime(uint64_t raw_timestamp)
    {
      // Convert CIB timestamp (16 ns ticks since Unix epoch) to
      // OPC-UA DateTime (100 ns ticks since Windows epoch 1601-01-01)
      
      // Step 1: Convert 16 ns ticks to seconds
      uint64_t seconds_since_unix_epoch = raw_timestamp / get_clock_frequency_hz();
      
      // Step 2: Add the epoch offset (1601 to 1970)
      int64_t seconds_since_windows_epoch = static_cast<int64_t>(seconds_since_unix_epoch) + get_epoch_offset_seconds();
      
      // Step 3: Convert seconds to 100 ns intervals
      // 1 second = 10,000,000 intervals of 100 ns
      uint64_t ua_datetime = static_cast<uint64_t>(seconds_since_windows_epoch) * 10000000ULL;
      
      // Step 4: Add sub-second precision
      // Remaining ticks after removing full seconds
      uint64_t remaining_ticks = raw_timestamp % get_clock_frequency_hz();
      // Convert remaining 16 ns ticks to 100 ns intervals: multiply by 100/16 = 6.25
      uint64_t subsecond_100ns = (remaining_ticks * 100) / 16;
      ua_datetime += subsecond_100ns;
      
      return ua_datetime;
    }

    uint64_t cib_time::from_ua_datetime(uint64_t ua_datetime)
    {
      // Convert OPC-UA DateTime (100 ns ticks since Windows epoch) to
      // CIB timestamp (16 ns ticks since Unix epoch)
      
      // Step 1: Convert 100 ns intervals to seconds
      uint64_t seconds_since_windows_epoch = ua_datetime / 10000000ULL;
      
      // Step 2: Subtract the epoch offset (1601 to 1970)
      int64_t seconds_since_unix_epoch = static_cast<int64_t>(seconds_since_windows_epoch) - get_epoch_offset_seconds();
      
      if (seconds_since_unix_epoch < 0)
      {
        throw std::runtime_error("OPC-UA DateTime is before Unix epoch (1970-01-01)");
      }
      
      // Step 3: Convert seconds to 16 ns intervals
      uint64_t raw_timestamp = static_cast<uint64_t>(seconds_since_unix_epoch) * get_clock_frequency_hz();
      
      // Step 4: Add sub-second precision
      // Remaining 100 ns intervals after removing full seconds
      uint64_t remaining_100ns = ua_datetime % 10000000ULL;
      // Convert 100 ns intervals to 16 ns ticks: multiply by 16/100 = 0.16
      uint64_t subsecond_ticks = (remaining_100ns * 16) / 100;
      raw_timestamp += subsecond_ticks;
      
      return raw_timestamp;
    }

  } // namespace util
} // namespace cib
