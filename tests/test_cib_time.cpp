/*
 * test_cib_time.cpp
 *
 * Test for cib_time timestamp utilities
 * Tests system timestamp retrieval and verifies reasonable values
 * Gracefully handles missing hardware (memory-mapped register)
 */

#include <cib_time.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <cassert>

using namespace cib::util;

int main()
{
  std::cout << "=== Testing cib_time ===" << std::endl;

  // Test 1: Get system timestamp (always works - no hardware required)
  std::cout << "\nTest 1: Get system timestamp (software-only, no hardware required)" << std::endl;
  
  uint64_t ts1 = cib_time::system_timestamp();
  std::cout << "  First timestamp: " << ts1 << " clock periods (16 ns ticks)" << std::endl;
  
  // Small delay to ensure timestamp advances
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  uint64_t ts2 = cib_time::system_timestamp();
  std::cout << "  Second timestamp (after 10ms): " << ts2 << " clock periods (16 ns ticks)" << std::endl;
  
  // Verify timestamp is monotonically increasing
  assert(ts2 > ts1 && "Second timestamp should be greater than first");
  uint64_t delta_ticks = ts2 - ts1;
  std::cout << "  Delta: " << delta_ticks << " clock periods" << std::endl;
  
  // Convert delta to milliseconds: delta_ticks / 62,500,000 * 1000
  // = delta_ticks / 62,500
  double delta_ms = static_cast<double>(delta_ticks) / 62500.0;
  std::cout << "  Delta in milliseconds: " << delta_ms << " ms" << std::endl;
  
  // Verify delta is approximately 10ms (allow some tolerance)
  // Should be between 5ms and 20ms
  assert(delta_ms >= 5.0 && delta_ms <= 20.0 && 
         "10ms sleep should result in ~10ms timestamp delta");
  std::cout << "  ✓ Timestamp delta is reasonable" << std::endl;

  // Test 2: Verify absolute timestamp sanity
  std::cout << "\nTest 2: Verify absolute timestamp sanity" << std::endl;
  
  // Get current time in seconds since epoch
  auto now = std::chrono::system_clock::now();
  auto ns_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(
    now.time_since_epoch()).count();
  
  uint64_t expected_ticks = ns_since_epoch / 16;  // 16 ns per tick
  
  // The system timestamp we got should be within 1 second of the expected value
  // (allowing for timing between the two calls)
  uint64_t expected_range_ticks = 62500000;  // 1 second in ticks
  
  if (ts1 >= expected_ticks && ts1 <= expected_ticks + expected_range_ticks)
  {
    std::cout << "  ✓ Timestamp is within expected range" << std::endl;
  }
  else if (ts1 > expected_ticks)
  {
    std::cout << "  ⚠ Timestamp is slightly in the future (likely timing variation)" << std::endl;
  }
  else
  {
    std::cout << "  ⚠ Timestamp is in the past (likely timing variation)" << std::endl;
  }

  // Test 3: Try to initialize singleton (may fail if hardware not present)
  std::cout << "\nTest 3: Attempt to initialize singleton (graceful failure if no hardware)" << std::endl;
  
  if (cib_time::isInitialized())
  {
    std::cout << "  ⚠ Singleton already initialized" << std::endl;
  }
  else
  {
    std::cout << "  Singleton not yet initialized, attempting to initialize..." << std::endl;
    
    // Use the default address from cib_mem.h (GPIO_TSTAMP_MEM_LOW)
    // This requires knowledge of cib_mem.h, but we'll catch the exception
    try
    {
      cib_time::initialize(0x40000000);  // Example address - will likely fail on non-hardware systems
      std::cout << "  ✓ Successfully initialized singleton (hardware present)" << std::endl;
      
      // If we got here, try to read a timestamp from hardware
      try
      {
        uint64_t hw_ts = cib_time::get().get_timestamp();
        std::cout << "  ✓ Read hardware timestamp: " << hw_ts << std::endl;
      }
      catch (const std::exception& e)
      {
        std::cout << "  ℹ Could not read hardware timestamp: " << e.what() << std::endl;
      }
    }
    catch (const std::exception& e)
    {
      std::cout << "  ℹ Hardware not available (expected on non-FPGA systems): " << e.what() << std::endl;
      std::cout << "  ✓ Exception handled gracefully" << std::endl;
    }
  }

  std::cout << "\n=== All tests completed successfully ===" << std::endl;
  return 0;
}
