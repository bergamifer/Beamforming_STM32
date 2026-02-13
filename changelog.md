# Changelog

## [2026-02-13] - Beamforming Delay Lines
- Replaced sine LUT with circular delay line beamforming pipeline
- Delay lines: delay_line_L[1024] + delay_line_R[1024] in RAM_D2 (ring buffer, power-of-2 mask)
- Process_ADC_Half(): deinterleaves stereo ADC RX into L/R delay lines
- Fill_Audio_Beamform(): per-channel delay/gain/mute from delay line, fixed-point gain (*gain>>8)
- beam_config_t: 32-channel local config (delays, gains, mutes, master_gain)
- ESP32 config handler: copies ds_config_t (16ch) into beam_config_t (first 16 slots)
- MAX_DELAY_SAMPLES=64 (~1.33ms @ 48kHz = ~45.7cm at 343 m/s)
- Dual beam architecture ready: delay_line_R populated but not yet read by output
- Removed: sine_lut[], Init_Sine_LUT(), Fill_Audio_Buffer(), math.h include
- Build: text=51520, bss=149232 (0 errors, 0 warnings)

## [2026-02-12] - 5 SAI Blocks (DS10 Config)
- Added SAI1_B (slave TX sync, PE3 AF6, DMA1_Stream3 req88) for channels 8-15
- Fixed SAI2_B pin: PD8 (wrong, SAI3) → PE11 (correct, SAI2_B AF10 sync)
- Added SAI3_B (master RX, PD8/PD9/PD10 AF6, DMA1_Stream4 req114) for ADC stereo
- 5 DMA streams: Stream0(SAI1_A), Stream1(SAI2_A), Stream2(SAI2_B), Stream3(SAI1_B), Stream4(SAI3_B RX)
- SAI3_B uses DMA_PERIPH_TO_MEMORY, SCB_InvalidateDCache for RX coherency
- Start order: slave before master (SAI1_B→SAI1_A, SAI2_B→SAI2_A, SAI3_B)
- PE3 LED removed (now SAI1_SD_B in DS10 config)
- DS9 pin audit: 6 SAI mapping errors found (see ds9_pin_audit.md)
- E-Paper pins relocated: PD8/9/10 → PD4/5/6 (freed for SAI3_B)
- Build: text=53048, bss=141128 (0 errors, 0 warnings)

## [2026-02-11] - UART Protocol & Non-blocking Print
- UART4 binary protocol for ESP32-STM32 communication (uart_protocol.c/h)
- State machine parser: byte-by-byte via RX interrupt, 9 states, XOR checksum
- ds_config_t: 52-byte packed struct (master_gain, delays[16], gains[16], mutes)
- ACK/NACK response to ESP32
- Config print to UART7 (minicom) for verification
- UART4 MSP init: PA0(TX), PA1(RX), AF8, IRQ priority 1
- Tested end-to-end: Flask -> ESP32 -> STM32 -> minicom, 0 errors, 0 underruns

## [2026-02-09] - CPU 480MHz Upgrade
- VOS0 overdrive: PLL1 M=5/N=192/P=2 = 480MHz SYSCLK
- Bus dividers: AHB/2=240MHz, APB1-4/2=120MHz, FLASH_LATENCY_4
- PCM1690 driver created (pcm1690.h/pcm1690.c)
- SAI1 changed from I2S stereo to TDM8 (FrameLength=256, 8 slots)
- Jitter bug fix: DWT reset race condition (atomic + sanity guard)

## [2026-02-07] - DMA Pipeline Optimization & E-Paper
- Deferred processing: ISR sets flag, main loop fills buffer
- Sine LUT replaces sinf() (~3 cycles/sample vs ~100)
- Cache coherence: SCB_CleanDCache_by_Addr after fill
- E-Paper SSD1681 driver (SPI2 TX-only, BUSY pull-down for optional)
- Init order: audio DMA starts BEFORE e-paper

## [2026-02-05] - DMA Stability Verification
- 60-second stability test: ~3000 callbacks, 0 errors
- Latency characterization with DWT cycle counter

## [2026-01-21] - Initial Design
- Hardware base defined, power supply calculated, SAI pinout designed
