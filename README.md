# Beamforming_STM32

STM32H743 audio beamforming firmware with **5 SAI blocks**, **32-channel TX**, **2-channel RX**, and per-channel delay/gain/mute control.

**Status**: ✅ Working — 0 errors, 0 underruns, 480MHz CPU, beamforming pipeline active

---

## Quick Start

### Hardware
- **MCU**: STM32H743VIT6 (ARM Cortex-M7, 480MHz, LQFP100)
- **ADC**: PCM1802 stereo (SAI3_B RX)
- **DACs**: 4× PCM1690 8-ch TDM (SAI1_A/B, SAI2_A/B → 32 speakers)
- **Board**: WeAct with DS10 pinout + optional SSD1681 e-paper display
- **Config**: ESP32 via UART4 binary protocol

### Build & Flash
```bash
cd firmware
make -j4              # Compile
make flash-stlink     # Flash via ST-Link
```

### Serial Monitor
```bash
minicom -D /dev/tty.usbmodem102 -b 115200
```

---

## Architecture

```
ADC Input (SAI3_B)
      ↓
Circular Delay Lines (1024 samples, L/R)
      ↓
Per-Channel Processing: delay → gain → mute
      ↓
32-Channel DAC Output (SAI1/SAI2 TDM)
      ↓
32 Speakers
```

### Key Features
- **DMA Circular Mode**: 10ms half-buffers, deferred fill
- **Latency**: ~20.6ms (dominated by DMA buffers)
- **Synchronization**: SAI1_B and SAI2_B share SCK/FS with masters
- **Config**: ESP32 control via UART4 (gains, delays, mutes, master level)
- **Monitoring**: DWT cycle counter for jitter analysis

---

## Pinout (DS10)

### SAI (Audio)
| Block | Master/Slave | Pins | AF | DMA | Ch |
|-------|---|---|---|---|---|
| SAI1_A | Master TX | PE4(FS) PE5(SCK) PE6(SD) | AF6 | Stream0 | 0-7 |
| SAI1_B | Slave TX | PE3(SD) | AF6 | Stream3 | 8-15 |
| SAI2_A | Master TX | PD12(FS) PD13(SCK) PD11(SD) | AF10 | Stream1 | 16-23 |
| SAI2_B | Slave TX | PE11(SD) | AF10 | Stream2 | 24-31 |
| SAI3_B | Master RX | PD8(SCK) PD9(SD) PD10(FS) | AF6 | Stream4 | ADC L+R |
| MCLK | All | PE2 | AF6 | — | 12.288 MHz |

### Serial & Other
| Peripheral | Pins | Function |
|---|---|---|
| UART7 | PA8(RX) PA15(TX) | Debug @ 115200 |
| UART4 | PA0(TX) PA1(RX) | ESP32 @ 115200 |
| I2C1 | PB8(SCL) PB9(SDA) | DAC config |
| SPI2 | PB13(SCK) PB15(MOSI) | E-Paper |
| E-Paper GPIO | PB12(CS) PD4(DC) PD5(RST) PD6(BUSY) | Display control |

---

## Clocks

| Clock | Frequency | Purpose |
|-------|-----------|---------|
| SYSCLK (PLL1) | 480 MHz | CPU (VOS0 overdrive) |
| HCLK | 240 MHz | AHB buses |
| APB | 120 MHz | Peripherals |
| PLL3P | 12.288 MHz | SAI1/2/3 MCLK (256 × 48 kHz) |

---

## Firmware Structure

```
firmware/
├── Core/Src/
│   ├── main.c                 # Beamforming, DMA callbacks, main loop
│   ├── stm32h7xx_hal_msp.c    # MSP init (5 SAI, 2 UART, I2C, SPI)
│   ├── stm32h7xx_it.c         # IRQ handlers
│   ├── uart_protocol.c        # ESP32 protocol parser
│   ├── pcm1690.c              # DAC driver
│   └── epaper.c               # E-Paper driver
├── Core/Inc/
│   ├── pin_definitions_ds10.h # DS10 corrected pins
│   ├── uart_protocol.h        # Binary protocol structs
│   ├── pcm1690.h              # DAC header
│   └── epaper.h               # E-Paper header
├── STM32H743XX_FLASH.ld       # Linker (.dma_buffer → RAM_D2)
└── Makefile
```

---

## Documentation

| Document | Purpose |
|---|---|
| [`docs/audio_architecture.md`](docs/audio_architecture.md) | Full beamforming pipeline & DAC config |
| [`uart-stm32-protocol.md`](uart-stm32-protocol.md) | ESP32 binary protocol specification |
| [`stm32/dma_bdma_config.md`](stm32/dma_bdma_config.md) | DMA streams & memory layout |
| [`memory_map.md`](memory_map.md) | RAM allocation (144 KB / 288 KB used) |
| [`changelog.md`](changelog.md) | Version history |

---

## Next Steps

- [ ] DS9 board testing (2 DACs, 16 channels verified)
- [ ] DS10 production test (full 32 channels)
- [ ] Dual-beam implementation (sum delay_line_L + delay_line_R)
- [ ] Calibration via cross-correlation (real delay compensation)
- [ ] Web dashboard (WebServer control)

---

**Author**: FBergami | **License**: (see project)
**Built**: Feb 2026 | **Target**: STM32H743VIT6 WeAct
