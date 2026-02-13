# DMA Configuration - WeAct STM32H743 Audio Project

**Last Updated:** 2026-02-13
**Status:** 5 DMA streams working (4 TX + 1 RX), beamforming pipeline active

---

## STM32H7 Memory Architecture for DMA

```
DTCMRAM (0x20000000, 128K) - DMA1/DMA2 NO PUEDEN ACCEDER
RAM_D2  (0x30000000, 288K) - DMA1/DMA2 SI (audio buffers, delay lines)
RAM_D3  (0x38000000, 64K)  - BDMA (no usado en DS10 — SAI4 descartado)
```

Buffers DMA requieren:
```c
__attribute__((section(".dma_buffer"), aligned(32)))
```

---

## Configuracion Actual: 5 DMA Streams

| Stream | SAI Block | DMAMUX Req | Direction | Mode | Channels | Buffer Size |
|--------|-----------|------------|-----------|------|----------|-------------|
| DMA1_Stream0 | SAI1_A TX | 87 | Mem→Per | Circular | ch 0-7 | 30,720 B |
| DMA1_Stream1 | SAI2_A TX | 89 | Mem→Per | Circular | ch 16-23 | 30,720 B |
| DMA1_Stream2 | SAI2_B TX | 90 | Mem→Per | Circular | ch 24-31 | 30,720 B |
| DMA1_Stream3 | SAI1_B TX | 88 | Mem→Per | Circular | ch 8-15 | 30,720 B |
| DMA1_Stream4 | SAI3_B RX | 114 | Per→Mem | Circular | ADC L+R | 7,680 B |

**Total bandwidth:** ~3.5 MB/s (de ~200 MB/s disponibles = 1.7%)

### Start Order (critico para sync)
1. SAI1_B (slave TX) — debe estar ready antes de master
2. SAI1_A (master TX) — arranca SAI1_B simultaneamente
3. SAI2_B (slave TX)
4. SAI2_A (master TX) — arranca SAI2_B
5. SAI3_B (master RX) — independiente

### Cache Coherency
- **TX buffers**: `SCB_CleanDCache_by_Addr()` despues de Fill_Audio_Beamform()
- **RX buffer**: `SCB_InvalidateDCache_by_Addr()` antes de Process_ADC_Half()

---

## DMA Callbacks y Beamforming Pipeline

```
DMA Half/Full Complete ISR → set deferred flag → main loop processes

Main loop order (cada 10ms):
  1. RX: Process_ADC_Half() → feeds delay_line_L/R
  2. TX: Fill_Audio_Beamform(buf, 480, ch_offset) × 4 blocks
  3. Cache clean/invalidate per buffer
```

Callbacks por segundo: 50 half + 50 full × 5 SAI blocks = 500 total

---

## DMAMUX Request IDs (RM0433 Table 122)

### DMAMUX1 (DMA1/DMA2)

| Request | Peripheral | Used |
|---------|------------|------|
| 87 | SAI1_A | Stream0 TX |
| 88 | SAI1_B | Stream3 TX |
| 89 | SAI2_A | Stream1 TX |
| 90 | SAI2_B | Stream2 TX |
| 113 | SAI3_A | — |
| 114 | SAI3_B | Stream4 RX |

### DMAMUX2 (BDMA - D3 domain)

| Request | Peripheral | Notes |
|---------|------------|-------|
| 15 | SAI4_A | Not used (SAI4_B SCK/FS not on LQFP100) |
| 16 | SAI4_B | Not used |

---

## Buffer Structure

```c
#define AUDIO_BUFFER_SAMPLES 480      // 10ms @ 48kHz
#define AUDIO_CHANNELS       8        // TDM8
#define AUDIO_BUFFER_SIZE    (480 * 8) // 3840 samples

// 4x TX buffers (double-buffered, each 30,720 bytes)
int32_t audio_buffer[3840 * 2];     // SAI1_A ch0-7
int32_t audio_buffer_1b[3840 * 2];  // SAI1_B ch8-15
int32_t audio_buffer_2a[3840 * 2];  // SAI2_A ch16-23
int32_t audio_buffer_2b[3840 * 2];  // SAI2_B ch24-31

// 1x RX buffer (double-buffered, stereo, 7,680 bytes)
int32_t audio_buffer_3b_rx[960 * 2]; // SAI3_B ADC L+R

// Delay lines (circular ring buffers, 4,096 bytes each)
int32_t delay_line_L[1024];  // beam 1
int32_t delay_line_R[1024];  // beam 2 (future)

// Total: ~144 KB in RAM_D2 (50% of 288 KB)
```
