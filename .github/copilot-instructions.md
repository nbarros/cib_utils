# CIB Utilities Codebase Instructions

## Architecture Overview

**CIB** (Calibration Interface Board) is part of the DUNE DAQ system. This codebase provides core utilities for detector readout, slow control integration, and hardware interfacing across multiple subsystems:

- **cib_utils**: Core readout and hardware control libraries (primary focus)
- **opcua-server**: OPC-UA slow control interface (Quasar framework)
- **LaserControl**: Laser calibration hardware interface
- **ndma**: Zynq DMA kernel module for high-speed data streaming

### Key Design: Handler/Reader Pattern

All detector data flows through a central `Handler` (network-facing control) → reader hierarchy:
- `Handler`: TCP listener receiving JSON config commands, manages threads
- `ReaderBase`: Simulation/test reader (base class for common TX plumbing)
- `ReaderAXIFIFO`: Primary reader for normal hardware operation
- Uses **lock-free queues** (`boost::lockfree::spsc_queue`) for thread-safe data staging
- Data transmission via Boost ASIO TCP to DAQ backend

**Key files**: [daq/Handler.cpp](../daq/Handler.cpp), [daq/ReaderBase.h](../daq/ReaderBase.h), [common/cib_data_fmt.h](../common/cib_data_fmt.h)

## Build System

All modules use **CMake 3.5+** with consistent patterns:
- Each subdirectory builds libraries or executables via `add_subdirectory()`
- Typical CMakeLists.txt links: `spdlog`, `Boost` (ASIO, lockfree), `nlohmann/json`
- All code uses **C++11** minimum (`cxx_std_11` required)
- Logging configured via `SPDLOG_ACTIVE_LEVEL` compile definition

**Build**: `mkdir build && cd build && cmake .. && make`

## Critical Developer Patterns

### 1. Configuration via JSON
- `Handler::config()` accepts JSON payloads: `{"detector_id": 0, "streams": [...]}`
- Uses `nlohmann::json` (header-only library in contrib/)
- Responses are JSON with `"status"` and `"feedback"` fields

### 2. Thread Lifecycle
- **Control thread** spawned in `Handler::init_listener()` runs `listen_task()`
- Uses `std::atomic<bool>` flags (`m_stop_running`, `m_is_running`) for synchronization
- **Blocking pattern**: Thread joins with timeout before destruction
- `WorkerThread` class in [common/WorkerThread.hpp](../common/WorkerThread.hpp) provides reusable helper

### 3. Memory Mapping & Hardware Access
- GPIO registers, FIFO buffers accessed via [common/cib_mem.h](../common/cib_mem.h) mappings
- Motor/trigger/PDTS hardware controlled through register writes
- Data format structs (e.g., `lbls_data_t`, `motor_t`) in [common/cib_data_fmt.h](../common/cib_data_fmt.h)

### 4. Simulation Mode
- Set `bool simulation = true` in Handler constructor to bypass hardware
- `ReaderBase` (simulation) vs. `ReaderAXIFIFO` (real) swapped at runtime
- Enables testing on non-Zynq systems

### 5. Socket Lifecycle & Shutdown Handshake
**Two independent TCP connections exist:**
- **Control socket** (port 8992): CIB server ← DAQ client (JSON config commands)
- **Data socket**: CIB client → DAQ server (detector readout)

**Proper shutdown sequence:**
1. DAQ sends `{"command":"stop_run"}` via control socket
2. CIB's `Handler::stop_run()` calls `Reader::stop_run()` → `term_transmitter()`
3. Data socket is closed, then response is sent back on control socket
4. DAQ receives response = proof that data socket has been closed
5. DAQ can now safely close its data receiver socket

**Key insight**: Response arrival on control socket IS the synchronization point — the response can only be sent after data socket closure completes.

## Integration Points

### DUNE DAQ Framework
`cibmodules/` plugin wraps CIB in DUNE DAQ:
- [cibmodules/plugins/CIBModule.cpp](../../dunedaq/work/sources/cibmodules/plugins/CIBModule.cpp) defines DAQ module interface
- Uses appmodel schema for configuration (see [appmodel/docs/README.md](../../dunedaq/work/sources/appmodel/docs/README.md))
- **Data in**: JSON config from DAQ manager → Handler
- **Data out**: Detector frames shipped via ASIO to DAQ backend

### OPC-UA Slow Control
`opcua-server/` exposes CIB state and commands:
- Motor positions, trigger rates, error status as OPC-UA variables
- Configuration pushed to CIB via ZMQ or direct socket

### Hardware Stack
- **ndma kernel module**: DMA setup for Zynq PCIe (cross-compiled ARM build required)
- **i2c module**: Peripheral control (laser power, motor command)
- **versaclock**: Clock configuration (deprecated, kept for reference)

## Logging & Debugging

- **spdlog**: Configured in each app's `main()`, see [apps/cib_daq_server.cpp](../apps/cib_daq_server.cpp)
- Set pattern: `spdlog::set_pattern("[%s:%!:%#][%^%L%$] [thread %t] %v")`
- Log level controlled by `SPDLOG_LEVEL` env var or `spdlog::set_level()`
- TRACE level enabled via `SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE` at compile time
- Common macros: `SPDLOG_INFO`, `SPDLOG_ERROR`, `SPDLOG_DEBUG`

## Testing & Workflows

### Quick Test
```bash
cd cib_utils && mkdir build && cd build && cmake .. && make -j4
./apps/cib_daq_server &  # Starts listening on port 8992
# Send JSON: echo '{"cmd": "config"}' | nc localhost 8992
```

### Simulation Mode
Build and run with `cib_daq_server` (auto-enables simulation for testing without hardware)

### Cross-compiling NDMA
```bash
source /opt/Xilinx/Vivado/2020.2/settings64.sh
export CROSS_COMPILE=arm-linux-gnueabihf-
make -C ndma/kernel ARCH=arm KDIR=/path/to/linux-xlnx
```

## Code Style & Conventions

- **Namespace**: All CIB code in `namespace cib` (or `dunedaq::cibmodules` for DAQ plugin)
- **Member variables**: `m_` prefix (e.g., `m_reader`, `m_control_thread`)
- **Delete copy/move**: All managers explicitly deleted (`= delete`)
- **RAII**: Use `std::unique_ptr`, `std::lock_guard` for resource cleanup
- **Comments**: File headers with creation date/author; complex sections documented

## Common Issues

1. **Compilation failures**: Check Boost version (ASIO, lockfree) and spdlog availability
2. **Socket timeouts**: Handler control timeout is `1000000 µs` (1 sec); adjust `m_control_timeout` if needed
3. **Thread join hangs**: Ensure `m_stop_running` is set before joining
4. **Memory access crashes**: Verify GPIO address mappings in [common/cib_constants.cpp](../common/cib_constants.cpp)

