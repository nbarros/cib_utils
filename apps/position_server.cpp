/*
 * position_server.cpp
 *
 *  Created on: Nov 7, 2024
 *      Author: Nuno Barros
 *
 *  ZMQ server that listens for position updates (3 signed integers)
 *  and writes them to memory-mapped motor control registers.
 *  Designed to run as a systemd service.
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/syslog_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <zmq.hpp>
#include <string>
#include <vector>
#include <cstring>
#include <unistd.h>

#include <mem_utils.h>
#include <cib_mem.h>
#include <cib_data_fmt.h>

using cib::cib_mem_t;
using cib::mapped_mem_t;
using cib::motor_t;
#define PAGE_SIZE 4096

// Global variables
cib_mem_t g_cib_mem;
int g_mem_fd = 0;
volatile std::atomic<bool> g_running(true);
cib::limits::motor_limits_t m_limits;
static bool g_simulate = false; // simulation mode flag

// Simulated motor registers (used only when g_simulate==true)
static uint32_t sim_motor_1_reg = 0;
static uint32_t sim_motor_2_reg = 0;
static uint32_t sim_motor_3_reg = 0;

// Function prototypes
int map_memory();
void clear_memory();
int motor_init_limits();
int set_motor_position(int32_t x, int32_t y, int32_t z);
void signal_handler(int signal);
void setup_logging(bool use_syslog);

// Signal handler for graceful shutdown
void signal_handler(int signal)
{
    spdlog::info("Received signal {}, shutting down...", signal);
    g_running = false;
}

// Setup logging (syslog for daemon mode, console otherwise)
void setup_logging(bool use_syslog)
{
    if (use_syslog)
    {
        auto syslog_sink = std::make_shared<spdlog::sinks::syslog_sink_mt>("position_server", LOG_PID, LOG_DAEMON, true);
        auto logger = std::make_shared<spdlog::logger>("position_server", syslog_sink);
        spdlog::set_default_logger(logger);
    }
    else
    {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        auto logger = std::make_shared<spdlog::logger>("position_server", console_sink);
        spdlog::set_default_logger(logger);
    }
    
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
}

// Initialize motor limits
int motor_init_limits()
{
    // These limits match the ones in cib_manager.cpp
    // You may need to adjust based on your specific hardware configuration
    m_limits.m1_min = -1000000;  // Motor 1 (RNN800/TSTAGE)
    m_limits.m1_max = 1000000;
    m_limits.m2_min = -1000000;  // Motor 2 (RNN600)
    m_limits.m2_max = 1000000;
    m_limits.m3_min = -65536;    // Motor 3 (LSTAGE)
    m_limits.m3_max = 65535;
    
    spdlog::info("Motor limits initialized: M1[{},{}] M2[{},{}] M3[{},{}]",
                 m_limits.m1_min, m_limits.m1_max,
                 m_limits.m2_min, m_limits.m2_max,
                 m_limits.m3_min, m_limits.m3_max);
    return 0;
}

// Map physical memory to virtual address space
int map_memory()
{
    if (g_simulate)
    {
        spdlog::info("Simulation mode enabled: skipping physical memory mapping");
        // Point the virtual addresses to our simulated registers so that any debug prints
        // still show a non-null address. We won't actually dereference these via reg_read/write
        // in simulation mode, but having addresses helps for consistency.
        g_cib_mem.gpio_motor_1.p_addr = 0;
        g_cib_mem.gpio_motor_1.v_addr = reinterpret_cast<uintptr_t>(&sim_motor_1_reg);
        g_cib_mem.gpio_motor_2.p_addr = 0;
        g_cib_mem.gpio_motor_2.v_addr = reinterpret_cast<uintptr_t>(&sim_motor_2_reg);
        g_cib_mem.gpio_motor_3.p_addr = 0;
        g_cib_mem.gpio_motor_3.v_addr = reinterpret_cast<uintptr_t>(&sim_motor_3_reg);
        return 0;
    }

    spdlog::info("Mapping motor control registers...");
    
    g_cib_mem.gpio_motor_1.p_addr = GPIO_MOTOR_1_MEM_LOW;
    g_cib_mem.gpio_motor_1.v_addr = cib::util::map_phys_mem(g_mem_fd, GPIO_MOTOR_1_MEM_LOW, GPIO_MOTOR_1_MEM_HIGH);
    spdlog::debug("MOTOR_1 virtual address: 0x{:X}", g_cib_mem.gpio_motor_1.v_addr);
    if (g_cib_mem.gpio_motor_1.v_addr == 0x0)
    {
        spdlog::critical("Failed to map MOTOR_1 memory at physical address 0x{:X}", GPIO_MOTOR_1_MEM_LOW);
        return -1;
    }
    
    g_cib_mem.gpio_motor_2.p_addr = GPIO_MOTOR_2_MEM_LOW;
    g_cib_mem.gpio_motor_2.v_addr = cib::util::map_phys_mem(g_mem_fd, GPIO_MOTOR_2_MEM_LOW, GPIO_MOTOR_2_MEM_HIGH);
    spdlog::debug("MOTOR_2 virtual address: 0x{:X}", g_cib_mem.gpio_motor_2.v_addr);
    if (g_cib_mem.gpio_motor_2.v_addr == 0x0)
    {
        spdlog::critical("Failed to map MOTOR_2 memory at physical address 0x{:X}", GPIO_MOTOR_2_MEM_LOW);
        return -1;
    }
    
    g_cib_mem.gpio_motor_3.p_addr = GPIO_MOTOR_3_MEM_LOW;
    g_cib_mem.gpio_motor_3.v_addr = cib::util::map_phys_mem(g_mem_fd, GPIO_MOTOR_3_MEM_LOW, GPIO_MOTOR_3_MEM_HIGH);
    spdlog::debug("MOTOR_3 virtual address: 0x{:X}", g_cib_mem.gpio_motor_3.v_addr);
    if (g_cib_mem.gpio_motor_3.v_addr == 0x0)
    {
        spdlog::critical("Failed to map MOTOR_3 memory at physical address 0x{:X}", GPIO_MOTOR_3_MEM_LOW);
        return -1;
    }
    
    spdlog::info("Memory mapping completed successfully");
    return 0;
}

// Clean up memory mappings
void clear_memory()
{
    if (g_simulate)
    {
        spdlog::info("Simulation mode: clearing simulated registers");
        sim_motor_1_reg = 0;
        sim_motor_2_reg = 0;
        sim_motor_3_reg = 0;
        return;
    }

    spdlog::info("Unmapping memory...");
    
    if (g_cib_mem.gpio_motor_1.v_addr != 0x0)
    {
        cib::util::unmap_mem(g_cib_mem.gpio_motor_1.v_addr, PAGE_SIZE);
    }
    if (g_cib_mem.gpio_motor_2.v_addr != 0x0)
    {
        cib::util::unmap_mem(g_cib_mem.gpio_motor_2.v_addr, PAGE_SIZE);
    }
    if (g_cib_mem.gpio_motor_3.v_addr != 0x0)
    {
        cib::util::unmap_mem(g_cib_mem.gpio_motor_3.v_addr, PAGE_SIZE);
    }
    
    if (g_mem_fd > 0)
    {
        close(g_mem_fd);
        g_mem_fd = 0;
    }
    
    spdlog::info("Memory cleanup completed");
}

// Set motor positions by writing to memory-mapped registers
// This function mimics the behavior of set_motor_init_position in cib_manager.cpp
int set_motor_position(int32_t x, int32_t y, int32_t z)
{
    spdlog::info("Setting motor positions: X={}, Y={}, Z={}", x, y, z);
    
    // Validate positions against limits
    if ((x < m_limits.m1_min) || (x > m_limits.m1_max))
    {
        spdlog::error("X position {} out of range [{},{}]", x, m_limits.m1_min, m_limits.m1_max);
        return -1;
    }
    if ((y < m_limits.m2_min) || (y > m_limits.m2_max))
    {
        spdlog::error("Y position {} out of range [{},{}]", y, m_limits.m2_min, m_limits.m2_max);
        return -1;
    }
    if ((z < m_limits.m3_min) || (z > m_limits.m3_max))
    {
        spdlog::error("Z position {} out of range [{},{}]", z, m_limits.m3_min, m_limits.m3_max);
        return -1;
    }
    
    // Motor 1 (X axis) - 22 bit position
    uintptr_t maddr = g_cib_mem.gpio_motor_1.v_addr;
    uint32_t mask = cib::util::bitmask(21, 0);
    uint32_t dir = (x >= 0) ? 0 : 1;
    uint32_t reg = (dir << 31) | cib::util::cast_from_signed(x, mask);
    spdlog::debug("Writing MOTOR_1 register: 0x{:08X}", reg);
    if (!g_simulate)
    {
        cib::util::reg_write(maddr, reg);
        // Activate the written data (pulse bit 30)
        uint32_t activation_mask = cib::util::bitmask(30, 30);
        cib::util::reg_write_mask_offset(maddr, 0x1, activation_mask, 30);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        cib::util::reg_write_mask_offset(maddr, 0x0, activation_mask, 30);
    }
    else
    {
        // In simulation, just store the register value
        sim_motor_1_reg = reg;
    }

    // Motor 2 (Y axis) - 22 bit position
    maddr = g_cib_mem.gpio_motor_2.v_addr;
    mask = cib::util::bitmask(21, 0);
    dir = (y >= 0) ? 0 : 1;
    reg = (dir << 31) | cib::util::cast_from_signed(y, mask);
    spdlog::debug("Writing MOTOR_2 register: 0x{:08X}", reg);
    if (!g_simulate)
    {
        cib::util::reg_write(maddr, reg);
        uint32_t activation_mask = cib::util::bitmask(30, 30);
        cib::util::reg_write_mask_offset(maddr, 0x1, activation_mask, 30);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        cib::util::reg_write_mask_offset(maddr, 0x0, activation_mask, 30);
    }
    else
    {
        sim_motor_2_reg = reg;
    }

    // Motor 3 (Z axis) - 17 bit position
    maddr = g_cib_mem.gpio_motor_3.v_addr;
    mask = cib::util::bitmask(16, 0);
    dir = (z >= 0) ? 0 : 1;
    reg = (dir << 31) | cib::util::cast_from_signed(z, mask);
    spdlog::debug("Writing MOTOR_3 register: 0x{:08X}", reg);
    if (!g_simulate)
    {
        cib::util::reg_write(maddr, reg);
        uint32_t activation_mask = cib::util::bitmask(30, 30);
        cib::util::reg_write_mask_offset(maddr, 0x1, activation_mask, 30);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        cib::util::reg_write_mask_offset(maddr, 0x0, activation_mask, 30);
    }
    else
    {
        sim_motor_3_reg = reg;
    }

    // Read back values for verification
    int32_t x_readback = 0, y_readback = 0, z_readback = 0;
    if (!g_simulate)
    {
        mask = cib::util::bitmask(21, 0);
        x_readback = cib::util::cast_to_signed(cib::util::reg_read(g_cib_mem.gpio_motor_1.v_addr), mask);
        y_readback = cib::util::cast_to_signed(cib::util::reg_read(g_cib_mem.gpio_motor_2.v_addr), mask);
        mask = cib::util::bitmask(16, 0);
        z_readback = cib::util::cast_to_signed(cib::util::reg_read(g_cib_mem.gpio_motor_3.v_addr), mask);
    }
    else
    {
        mask = cib::util::bitmask(21, 0);
        x_readback = cib::util::cast_to_signed(sim_motor_1_reg, mask);
        y_readback = cib::util::cast_to_signed(sim_motor_2_reg, mask);
        mask = cib::util::bitmask(16, 0);
        z_readback = cib::util::cast_to_signed(sim_motor_3_reg, mask);
    }
    
    spdlog::info("Position set successfully. Readback: X={}, Y={}, Z={}", x_readback, y_readback, z_readback);
    
    return 0;
}

int main(int argc, char* argv[])
{
    // Parse command line arguments
    std::string bind_address = "tcp://*:5555";
    bool use_syslog = false;
    bool verbose = false;
    bool simulate = false;
    
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == "--syslog")
        {
            use_syslog = true;
        }
        else if (arg == "--verbose" || arg == "-v")
        {
            verbose = true;
        }
        else if (arg == "--bind" && i + 1 < argc)
        {
            bind_address = argv[++i];
        }
        else if (arg == "--simulate" || arg == "--sim")
        {
            simulate = true;
        }
        else if (arg == "--help" || arg == "-h")
        {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --bind <address>   ZMQ bind address (default: tcp://*:5555)\n"
                      << "  --syslog           Use syslog for logging (for systemd)\n"
                      << "  --verbose, -v      Enable verbose (debug) logging\n"
                      << "  --simulate, --sim  Run in simulation mode (no /dev/mem access)\n"
                      << "  --help, -h         Show this help message\n";
            return 0;
        }
    }
    
    // Setup logging
    setup_logging(use_syslog);
    if (verbose)
    {
        spdlog::set_level(spdlog::level::debug);
    }
    
    spdlog::info("Position Server starting...");
    spdlog::info("Bind address: {}", bind_address);
    if (simulate) { spdlog::info("Simulation mode requested"); }
    
    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Set global simulation flag before any initialization that depends on it
    g_simulate = simulate;

    // Initialize motor limits
    if (motor_init_limits() != 0)
    {
        spdlog::critical("Failed to initialize motor limits");
        return 1;
    }
    
    // Map memory
    if (map_memory() != 0)
    {
        spdlog::critical("Failed to map memory. Check permissions and hardware.");
        return 1;
    }
    
    // Initialize ZMQ context and socket
    zmq::context_t context(1);
    zmq::socket_t socket(context, zmq::socket_type::rep);
    
    try
    {
        socket.bind(bind_address);
        spdlog::info("Server listening on {}", bind_address);
    }
    catch (const zmq::error_t& e)
    {
        spdlog::critical("Failed to bind ZMQ socket: {}", e.what());
        clear_memory();
        return 1;
    }
    
    // Set socket timeout to allow checking g_running flag
    socket.set(zmq::sockopt::rcvtimeo, 1000);  // 1 second timeout
    
    spdlog::info("Position Server ready to receive position updates");
    
    // Main server loop
    while (g_running)
    {
        zmq::message_t request;
        
        try
        {
            // Try to receive a message (will timeout after 1 second)
            auto result = socket.recv(request, zmq::recv_flags::none);
            
            if (!result)
            {
                // Timeout or no message, continue loop to check g_running
                continue;
            }
            
            // Parse the received data
            // Support two formats: JSON and binary (3 x int32_t)
            
            if (request.size() == 12)  // Binary format: 3 x 32-bit integers
            {
                int32_t positions[3];
                std::memcpy(positions, request.data(), 12);
                
                int32_t x = positions[0];
                int32_t y = positions[1];
                int32_t z = positions[2];
                
                spdlog::debug("Received binary position: X={}, Y={}, Z={}", x, y, z);
                
                // Set motor positions
                int result = set_motor_position(x, y, z);
                
                // Send reply
                std::string reply_msg;
                if (result == 0)
                {
                    reply_msg = "OK";
                }
                else
                {
                    reply_msg = "ERROR";
                }
                
                zmq::message_t reply(reply_msg.size());
                std::memcpy(reply.data(), reply_msg.data(), reply_msg.size());
                socket.send(reply, zmq::send_flags::none);
            }
            else  // Assume JSON format
            {
                std::string msg_str(static_cast<char*>(request.data()), request.size());
                spdlog::debug("Received JSON message: {}", msg_str);
                
                // Simple JSON parsing (you might want to use a proper JSON library)
                // Expected format: {"x": 100, "y": -50, "z": 200}
                int32_t x = 0, y = 0, z = 0;
                bool parse_ok = true;
                
                try
                {
                    size_t x_pos = msg_str.find("\"RNN800\"");
                    size_t y_pos = msg_str.find("\"RNN600\"");
                    size_t z_pos = msg_str.find("\"LSTAGE\"");
                    
                    if (x_pos != std::string::npos)
                    {
                        size_t colon = msg_str.find(":", x_pos);
                        size_t comma = msg_str.find(",", colon);
                        if (comma == std::string::npos) comma = msg_str.find("}", colon);
                        x = std::stoi(msg_str.substr(colon + 1, comma - colon - 1));
                    }
                    else
                    {
                        parse_ok = false;
                    }
                    
                    if (y_pos != std::string::npos)
                    {
                        size_t colon = msg_str.find(":", y_pos);
                        size_t comma = msg_str.find(",", colon);
                        if (comma == std::string::npos) comma = msg_str.find("}", colon);
                        y = std::stoi(msg_str.substr(colon + 1, comma - colon - 1));
                    }
                    else
                    {
                        parse_ok = false;
                    }
                    
                    if (z_pos != std::string::npos)
                    {
                        size_t colon = msg_str.find(":", z_pos);
                        size_t comma = msg_str.find(",", colon);
                        if (comma == std::string::npos) comma = msg_str.find("}", colon);
                        z = std::stoi(msg_str.substr(colon + 1, comma - colon - 1));
                    }
                    else
                    {
                        parse_ok = false;
                    }
                }
                catch (const std::exception& e)
                {
                    spdlog::error("Failed to parse JSON message: {}", e.what());
                    parse_ok = false;
                }
                
                if (parse_ok)
                {
                    spdlog::debug("Parsed JSON position: X={}, Y={}, Z={}", x, y, z);
                    
                    // Set motor positions
                    int result = set_motor_position(x, y, z);
                    
                    // Send JSON reply
                    std::string reply_msg;
                    if (result == 0)
                    {
                        reply_msg = "{\"status\": \"OK\", \"RNN800\": " + std::to_string(x) + 
                                    ", \"RNN600\": " + std::to_string(y) + 
                                    ", \"LSTAGE\": " + std::to_string(z) + "}";
                    }
                    else
                    {
                        reply_msg = "{\"status\": \"ERROR\", \"message\": \"Position out of range\"}";
                    }
                    
                    zmq::message_t reply(reply_msg.size());
                    std::memcpy(reply.data(), reply_msg.data(), reply_msg.size());
                    socket.send(reply, zmq::send_flags::none);
                }
                else
                {
                    // Send error reply
                    std::string reply_msg = "{\"status\": \"ERROR\", \"message\": \"Invalid JSON format\"}";
                    zmq::message_t reply(reply_msg.size());
                    std::memcpy(reply.data(), reply_msg.data(), reply_msg.size());
                    socket.send(reply, zmq::send_flags::none);
                }
            }
        }
        catch (const zmq::error_t& e)
        {
            if (e.num() == EAGAIN)
            {
                // Timeout, continue loop
                continue;
            }
            else
            {
                spdlog::error("ZMQ error: {}", e.what());
            }
        }
        catch (const std::exception& e)
        {
            spdlog::error("Error processing message: {}", e.what());
            
            // Send error reply
            std::string reply_msg = "ERROR";
            zmq::message_t reply(reply_msg.size());
            std::memcpy(reply.data(), reply_msg.data(), reply_msg.size());
            socket.send(reply, zmq::send_flags::none);
        }
    }
    
    // Cleanup
    spdlog::info("Shutting down Position Server...");
    socket.close();
    context.close();
    clear_memory();
    
    spdlog::info("Position Server stopped");
    return 0;
}
