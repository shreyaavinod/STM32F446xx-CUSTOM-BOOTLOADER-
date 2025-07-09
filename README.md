# 🔧 STM32 Custom UART Bootloader

This (ongoing) project implements a **custom bootloader** for STM32 microcontrollers (tested on STM32F446RE) that allows firmware upgrades over UART. The bootloader communicates with a host system (e.g., a PC) via USART2, enabling operations like reading chip ID, erasing flash, writing to memory, reading protection levels, and jumping to the main application.

---

## 🚀 Features

- ✅ UART-based command interface (USART2)
- ✅ Host command packet with CRC32 verification
- ✅ Button-based mode selection (Bootloader / User App)
- ✅ Jumps to application stored in Flash Sector 2
- ✅ Debug prints via USART3 (`printf1()`)
- ✅ Commands supported:
  - `0x51`: Get bootloader version
  - `0x52`: Get list of supported commands
  - `0x53`: Get MCU chip ID
  - `0x54`: Get RDP (Read Protection) level
  - `0x55`: Jump to user application
  - `0x56`: Flash erase
  - `0x57`: Memory write
  - `0x58`: Enable R/W protection
  - `0x59`: Memory read
  - `0x5A`: Read sector protection status
  - `0x5B`: OTP read
  - `0x5C`: Disable R/W protection

---

## 📁 Project Structure
stm32-bootloader/
├── Core/
│ ├── Src/
│ │ └── main.c # Bootloader core logic
│ └── Inc/
│ └── main.h
├── Drivers/ # HAL Drivers
├── bootloader_commands.c/.h # (recommended for modular handlers)
├── README.md


---

## ⚙️ How It Works

- On power-up, the bootloader checks the **state of a GPIO pin** (PC13).
  - If **pressed**, enters **bootloader mode** and listens for commands via UART.
  - If **not pressed**, jumps to the **user application** at Flash Sector 2.
- Incoming command packets are verified using **CRC32**.
- Responds with **ACK** (0xA5) and optional data or **NACK** (0x7F).

---

## 📦 Host Command Format

Each packet from the host follows this structure:

| Byte | Description                    |
|------|--------------------------------|
| 0    | Length of remaining bytes (N)  |
| 1    | Command code                   |
| 2... | Parameters (optional)          |
| N-3 to N | 4-byte CRC32 (little endian) |

> ⚠️ The CRC is computed over all bytes except the CRC itself.

---

## 🛠️ Requirements

- STM32F4xx MCU (e.g., STM32F446RE)
- STM32CubeIDE
- USB to UART module
- UART terminal or Python host tool

---

## 🧰 Debugging

- All debug messages (e.g., CRC verification, command execution) are sent over **USART3** using a custom `printf1()` function.

---




