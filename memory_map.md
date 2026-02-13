# STM32H743 Memory Layout - Uso Actual (2026-02-13)

## RAM Distribution

| Region | Address | Size | DMA Access | Uso en proyecto |
|--------|---------|------|------------|-----------------|
| DTCMRAM | 0x20000000 | 128 KB | NO (CPU only) | Stack, variables, beam_config_t |
| AXI SRAM | 0x24000000 | 512 KB | MDMA only | No usado |
| RAM_D2 (SRAM1+2+3) | 0x30000000 | 288 KB | DMA1/DMA2 | Audio buffers, delay lines, e-paper |
| RAM_D3 (SRAM4) | 0x38000000 | 64 KB | BDMA | No usado (SAI4 descartado en DS10) |

## RAM_D2 (0x30000000) - Linker section `.dma_buffer`

| Buffer | Size (bytes) | Purpose |
|--------|-------------|---------|
| audio_buffer (SAI1_A) | 30,720 | TDM8 TX ch0-7 (480 x 8 x 2 halves x 4B) |
| audio_buffer_1b (SAI1_B) | 30,720 | TDM8 TX ch8-15 |
| audio_buffer_2a (SAI2_A) | 30,720 | TDM8 TX ch16-23 |
| audio_buffer_2b (SAI2_B) | 30,720 | TDM8 TX ch24-31 |
| audio_buffer_3b_rx (SAI3_B) | 7,680 | I2S RX ADC stereo (480 x 2 x 2 x 4B) |
| delay_line_L | 4,096 | Circular delay line beam 1 (1024 x 4B) |
| delay_line_R | 4,096 | Circular delay line beam 2 (1024 x 4B) |
| epd_buffer | 5,000 | E-Paper 200x200 / 8 |
| **Total** | **~144 KB** | **De 288 KB disponibles (50%)** |

## DTCMRAM (0x20000000)

| Variable | Size | Notes |
|----------|------|-------|
| beam_config_t | ~100 B | 32-ch delays/gains/mutes |
| ds_config_t (ESP32) | 52 B | Received from UART4 |
| Stack | ~8 KB | Default |
| Variables globales | ~2 KB | Counters, flags, handles |

## Regla Critica

**DMA1/DMA2 NO pueden acceder a DTCMRAM (0x20000000)**

Todo buffer fuente/destino de DMA debe estar en RAM_D2:
```c
__attribute__((section(".dma_buffer"), aligned(32)))
```

`aligned(32)` para coherencia de D-Cache (lineas de 32 bytes en Cortex-M7).

Cache:
- **TX buffers**: `SCB_CleanDCache_by_Addr()` despues de escribir (CPU→RAM→DMA)
- **RX buffers**: `SCB_InvalidateDCache_by_Addr()` antes de leer (DMA→RAM→CPU)
