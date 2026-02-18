# AH-64D MPD (DCS) — STM32F407 USB HID Controller

This project is an AH-64D Apache **MPD (Multi-Purpose Display) button panel** for **DCS**, implemented as a **driverless USB HID joystick** using an **STM32F407VGT6** board (instead of Arduino/Leo Bodnar).

- **Target sim:** DCS World (AH-64D)
- **USB class:** Custom HID (Joystick)
- **Inputs:** MPD buttons + rotary encoders (exposed as axes for game compatibility)
- **MCU board used:** STM32F407VGT6 / STM32F4XX-M “DevEBox STM32F4xx” style 
commonly found on AliExpress
  - Similar board with an STM32F407V**E**T instead of STM32F407V**G**T: 
  https://stm32-base.org/boards/STM32F407VET6-STM32F4XX-M

---

## Credits / Mechanical Design

All 3D printer files and the physical build guide are from:

**MilKris666/AH-64D-Apache-MPD-Multi-Purpose-Display**  
https://github.com/MilKris666/AH-64D-Apache-MPD-Multi-Purpose-Display

I made some alterations to the original .stl files to accomodate the STM's mount
pattern and a 100x100mm vesa style mount. I would be happy to share these with 
you with permission from the above github user as they are a modification of 
their design, and they deserve your financial support for their hard work.

This repo focuses on the **electronics + firmware** using STM32F407.

---

## Features

- **Plug-and-play HID** on Windows and Linux (no custom drivers)
- Buttons wired **active-low** using internal pull-ups
- Two rotary encoders read via **TIM3 / TIM4 encoder mode**
- Encoders are mapped to X/Y absolute axes (0–255)
- Optional DFU/bootloader entry by pressing and holding the **VID** and **BRT** 
buttons in for 5 seconds. 

---

## Hardware

### MCU Board
- STM32F407VGT6 (DevEBox STM32F4xx)
- USB FS device on **PA11 (DM)** / **PA12 (DP)**
- SWD debugging on **PA13/PA14**

### Rotary Encoders (EC11-style)
- **VID encoder:** TIM3 CH1/CH2 on **PA6/PA7**
- **BRT encoder:** TIM4 CH1/CH2 on **PB6/PB7**
- Both are configured in CubeMX as: `TIM_ENCODERMODE_TI12` with input filters

### Other Peripherals
- SPI3 for external flash (optional/if populated):
  - SCK **PB3**, MISO **PB4**, MOSI **PB5**
  - CS **PA15** (`FLASH_CS`)
- USART3 (optional debug/telemetry):
  - TX **PD8**, RX **PD9**
- LED output:
  - **PD2** (`LED1`)

---

## Wiring / Pinout (from `ah64.ioc`)

All button inputs below are configured as **GPIO input + pull-up** (active-low), 
with the exception of the onboard K0 connected to PA0 which is an active-high 
button.
```c                                                                           
.               PC5   PE2 PE3 PE4 PE5 PE6 PC13   PD11
           PB0   |    |   |   |   |   |   |      /
            \   VID   T1  T2  T3  T4  T5  T6   DAY
             BRT______________________________  NT--PD12
                |                            | MONO--PD13
        PC2--L1 |                            | R1--PE14
        PC3--L2 |                            | R2--PE15
        PA1--L3 |                            | R3--PB10
        PA2--L4 |                            | R4--PB11
        PA3--L5 |                            | R5--PB12
        PA4--L6 |                            | R6--PB13
        PA5--L7 |                            | R7--PB14
        PC4--L8 |                            | R8--PD14
                |                            | R9--PD15
.               ------------------------------
                 B1  B2  B3  B4  B5   B6   B7   B8
                 |   |   |   |   |    |    |    |
                PB1 PE7 PE8 PE9 PE10 PE11 PE12 PE13
```
---

## HID Report / Controls

This project enumerates as a **Joystick** HID device.
- **Buttons:** 36 buttons packed into 5 bytes
- **Encoders:** mapped to **ABS X/Y axes (0–255)** for game compatibility

---

## Build & Flash

### Tooling
- STM32CubeMX / STM32CubeIDE (or Makefile toolchain)
- ST-LINK for debugging/flashing (recommended)
- Optional DFU workflow 

### Typical steps
1. Clone down repo 
`git clone --recurse-submodules git@github.com:zuidec/ah64-mpd.git`
2. Optionally modify the `.ioc` in CubeMX (or use the committed generated 
   project).
3. Building:
    a. Build the firmware with arm-none-eabi toolchain from either STM's 
    programs or  your distribution's package manager.
    b. Import the project as a STM32 Makefile project and build it inside 
    CubeIDE
4. Flash using ST-LINK or DFU bootloader.
5. Further firmware updates can be done by holding the **VID** and **BRT** 
   buttons for 5 seconds to enter the DFU bootloader.

---

## License

Firmware in this repo: 
- GNU GPLv3 license

Mechanical files/build guide: see upstream repository’s license:
https://github.com/MilKris666/AH-64D-Apache-MPD-Multi-Purpose-Display

