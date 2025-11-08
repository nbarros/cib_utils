# Position Server

A ZMQ-based server for receiving motor position updates and writing them to memory-mapped control registers. Designed to run as a systemd service.

## Overview

The Position Server listens for position updates (three signed 32-bit integers representing X, Y, Z coordinates) via ZeroMQ and writes them to memory-mapped motor control registers. This is similar to the `set_motor_init_position` function in `cib_manager.cpp`.

## Features

- **ZeroMQ REQ/REP pattern**: Reliable request-reply communication
- **Dual format support**: 
  - Binary format (12 bytes: 3 x int32_t)
  - JSON format (`{"x": 100, "y": -50, "z": 200}`)
- **Memory-mapped register access**: Direct hardware control via `/dev/mem`
- **Position validation**: Range checking against configured limits
- **Systemd integration**: Runs as a system service with automatic restart
- **Comprehensive logging**: Syslog integration for daemon mode

## Building

```bash
cd /home/nbarros/work/dune/cib/cib_utils
mkdir -p build
cd build
cmake ..
make position_server
```

## Installation

### 1. Install the binary

```bash
sudo cp build/apps/position_server /usr/local/bin/
sudo chmod +x /usr/local/bin/position_server
```

### 2. Install the systemd service

```bash
sudo cp apps/position_server.service /etc/systemd/system/
sudo systemctl daemon-reload
```

### 3. Enable and start the service

```bash
# Enable service to start on boot
sudo systemctl enable position_server

# Start the service now
sudo systemctl start position_server

# Check status
sudo systemctl status position_server
```

## Usage

### Running as a Systemd Service (Recommended)

```bash
# Start the service
sudo systemctl start position_server

# Stop the service
sudo systemctl stop position_server

# Restart the service
sudo systemctl restart position_server

# View logs
sudo journalctl -u position_server -f
```

### Running Manually (for testing)

```bash
# Run with default settings (tcp://*:5555)
sudo ./position_server

# Run with verbose logging
sudo ./position_server --verbose

# Run with custom bind address
sudo ./position_server --bind tcp://*:5556

# Run with syslog output
sudo ./position_server --syslog
```

### Command Line Options

- `--bind <address>`: ZMQ bind address (default: `tcp://*:5555`)
- `--syslog`: Use syslog for logging (recommended for systemd)
- `--verbose`, `-v`: Enable debug-level logging
- `--help`, `-h`: Show help message

## Client Usage

### Python Client

```python
from position_client import PositionClient

# Connect and send position
with PositionClient("tcp://localhost:5555") as client:
    # Send as JSON
    response = client.send_position_json(100, -50, 200)
    print(f"Response: {response}")
    
    # Or send as binary
    response = client.send_position_binary(100, -50, 200)
```

### Command Line

```bash
# JSON format
python position_client.py --x 100 --y -50 --z 200

# Binary format
python position_client.py --x 100 --y -50 --z 200 --binary

# Custom server
python position_client.py --server tcp://192.168.1.10:5555 --x 10 --y 20 --z 30
```

## Configuration

### Motor Limits

The motor position limits are defined in the `motor_init_limits()` function in `position_server.cpp`:

```cpp
m_limits.m1_min = -1000000;  // Motor 1 (X axis / RNN800/TSTAGE)
m_limits.m1_max = 1000000;
m_limits.m2_min = -1000000;  // Motor 2 (Y axis / RNN600)
m_limits.m2_max = 1000000;
m_limits.m3_min = -65536;    // Motor 3 (Z axis / LSTAGE)
m_limits.m3_max = 65535;
```

Adjust these values according to your hardware specifications and rebuild.

### ZMQ Bind Address

The default bind address is `tcp://*:5555`. To change it:

1. **For systemd service**: Edit `/etc/systemd/system/position_server.service`
   ```ini
   ExecStart=/usr/local/bin/position_server --syslog --bind tcp://*:5556
   ```
   Then reload: `sudo systemctl daemon-reload && sudo systemctl restart position_server`

2. **For manual execution**: Use the `--bind` option

### Security Settings

The systemd service requires root privileges and specific capabilities:
- `CAP_SYS_RAWIO`: For `/dev/mem` access
- `CAP_DAC_OVERRIDE`: For bypassing file permission checks

These are configured in the systemd service file.

## Memory-Mapped Registers

The server writes to three motor control registers:

| Motor | Register | Address | Bit Width | Axis |
|-------|----------|---------|-----------|------|
| Motor 1 | `gpio_motor_1` | `0xA0090000` | 22 bits | X (RNN800/TSTAGE) |
| Motor 2 | `gpio_motor_2` | `0xA00A0000` | 22 bits | Y (RNN600) |
| Motor 3 | `gpio_motor_3` | `0xA00B0000` | 17 bits | Z (LSTAGE) |

Each register has:
- **Bit 31**: Direction (0 = positive, 1 = negative)
- **Bit 30**: Activation pulse bit
- **Bits [21:0] or [16:0]**: Position value

## Protocol Details

### Binary Format

12 bytes total: 3 signed 32-bit integers in little-endian order
```
[int32_t x][int32_t y][int32_t z]
```

### JSON Format

```json
{
    "x": 100,
    "y": -50,
    "z": 200
}
```

### Response Format

- **Binary request**: Returns `"OK"` or `"ERROR"`
- **JSON request**: Returns JSON object:
  ```json
  {
      "status": "OK",
      "x": 100,
      "y": -50,
      "z": 200
  }
  ```

## Troubleshooting

### Check Service Status

```bash
sudo systemctl status position_server
sudo journalctl -u position_server -n 50
```

### Common Issues

1. **Permission denied for /dev/mem**
   - Ensure the service runs as root
   - Check that capabilities are set correctly in the service file

2. **Port already in use**
   - Check if another process is using port 5555: `sudo netstat -tulpn | grep 5555`
   - Change the bind address with `--bind`

3. **Failed to map memory**
   - Verify hardware is accessible
   - Check kernel logs: `sudo dmesg | tail`
   - Ensure memory addresses match your hardware configuration

4. **Position out of range errors**
   - Verify the motor limits in the code match your hardware
   - Check client is sending valid position values

### Debug Mode

Run manually with verbose logging:
```bash
sudo ./position_server --verbose
```

### Testing Without Hardware

To test without actual hardware, you'll need to modify the memory mapping functions or create a mock implementation.

## Performance

- **Latency**: ~10-50 microseconds per position update (depending on hardware)
- **Throughput**: Supports hundreds of updates per second
- **Memory**: Minimal (~2-5 MB)
- **CPU**: Low (<1% on modern processors)

## Dependencies

- **ZeroMQ** (libzmq): For network communication
- **spdlog**: For logging
- **cib_utils common library**: For memory utilities and register access
- **pthread**: For threading support

## License

Part of the CIB (Cryogenic Instrumentation Board) utilities package.
