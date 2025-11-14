# JSON Modbus Server

A lightweight Modbus server implementation with JSON configuration support, written in C.

## Features

- **Modbus RTU & TCP Support**: Configure via modbus-rtu.h and modbus-tcp.h
- **JSON Configuration**: Use modbus_config.json for server settings
- **cJSON Library**: Integrated JSON parsing and manipulation
- **Unit Tests**: Comprehensive test suite with Unity framework

## Building

### Prerequisites

- GCC compiler
- libmodbus library
- GNU Make

### Compile

```bash
make all
```

### Clean

```bash
make clean
```

## Configuration

Edit `modbus_config.json` to configure:
- Modbus protocol settings (RTU/TCP)
- Device parameters
- Server behavior

## Usage

Run the server:

```bash
./json-modbus-server
```

For unit tests:

```bash
./unit-test-server
```

## Project Structure

- `json-modbus-server.c` - Main server implementation
- `unit-test-server.c` - Unit tests
- `modbus_config.json` - Configuration file
- `modbus.h`, `modbus-rtu.h`, `modbus-tcp.h` - Modbus protocol headers
- `cJSON/` - JSON library (embedded)

## License

This project is released under the **MIT License**. See [LICENSE](LICENSE) file for details.

### Third-party Libraries

- **cJSON**: Licensed under MIT License (see [cJSON/LICENSE](cJSON/LICENSE))
- **libmodbus**: Licensed under LGPL (see modbus headers)

## Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## Support

For issues and questions, please open an issue on the GitHub repository.

---
