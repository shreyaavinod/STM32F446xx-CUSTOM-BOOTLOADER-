# 🚀 STM32 Custom UART Bootloader

This project is a fully functional **custom UART bootloader** for the **STM32F4 series (tested on STM32F446RE)**. It allows secure, firmware-level control over memory operations such as flash erase, memory write, read/write protection, and user application jumps — all through UART.

---

## 📋 Features

- ✅ **Bootloader Entry via Button Press** (GPIOC Pin 13)
- ✅ **UART Command Protocol** (via USART2)
- ✅ **Flash Erase** (sector or mass erase)
- ✅ **Flash Programming** (byte-wise via HAL_FLASH_Program)
- ✅ **CRC Verification** of command packets
- ✅ **Read/Write Protection** using Option Bytes
- ✅ **Jump to User Application** in Sector 2
- ✅ **Command Debug Output** via USART3 (`printf1()`)
- ✅ **Provide a user application that: Initializes peripherals and Confirms successful execution after bootloader handoff

---

## 📦 Supported Bootloader Commands

| Command Code | Command Name             | Description                          |
|--------------|--------------------------|--------------------------------------|
| `0x51`       | `BL_GET_VER`             | Get bootloader version               |
| `0x52`       | `BL_GET_HELP`            | Get supported command list           |
| `0x53`       | `BL_GET_CID`             | Get MCU Chip ID                      |
| `0x54`       | `BL_GET_RDP_STATUS`      | Get current Read Protection Level    |
| `0x55`       | `BL_GO_TO_ADDR`          | Jump to specified address            |
| `0x56`       | `BL_FLASH_ERASE`         | Erase flash memory sectors           |
| `0x57`       | `BL_MEM_WRITE`           | Program flash with new firmware      |
| `0x58`       | `BL_EN_R_W_PROTECT`      | Enable Read/Write or Write protection|
| `0x59`       | `BL_MEM_READ`            | (To Be Implemented)                  |
| `0x5A`       | `BL_READ_SECTOR_STATUS`  | Check sector protection bits         |
| `0x5B`       | `BL_OTP_READ`            | (To Be Implemented)                  |
| `0x5C`       | `BL_DIS_R_W_PROTECT`     | Disable all sector protections       |

---

## 📂 Project Structure

```bash
STM32-Bootloader-Project/
├── Bootloader/
│   ├── Core/Src/main.c
│   ├── bootloader.c/.h
│   └── ...
├── User_Application/
│   ├── Core/Src/main.c
│   └── ...
└── README.md


---

## 🔧 How It Works

1. **Boot Decision:**
   - If `GPIOC Pin 13` is **pressed** → Bootloader mode
   - Else → Jump to user app in Sector 2

2. **UART Protocol:**
   - First byte: Total length (excluding length byte)
   - Next bytes: Command + arguments
   - Last 4 bytes: CRC32 checksum

3. **Sector & Flash Config:**
   - Flash Write & Erase via `HAL_FLASH_Unlock()` + `HAL_FLASH_Program`
   - Option Byte settings via `OPTCR_BYTEx_ADDRESS`

4. **Function Jump:**
   - MSP and reset handler of user app are extracted from address `0x08008000` (sector 2 base)

---

## 💻 UART Interface

- **USART2 (PA2 - TX, PA3 - RX)**: Bootloader command interface
- **USART3 (PB10 - TX)**: Debug messages via `printf1()`

---

## 🛠️ Flash Write Format (Example)

**Command:** `BL_MEM_WRITE`  
**Payload Format:**

## 🚦 To-Do / Future Improvements

- [ ] Implement `BL_MEM_READ` & `BL_OTP_READ`
- [ ] Timeout auto-jump to user app
- [ ] LED indication for status
- [ ] Add Python CLI bootloader flasher

---
