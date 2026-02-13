# WeAct STM32H743 - Audio Beamforming: Guia de Estudio

**Objetivo:** Validar audio beamforming pipeline (32ch TX + 2ch RX) con DMA circular en WeAct antes de implementar en DS v9/v10.

**Estado actual:** CPU 480MHz, 5 SAI blocks (4 TX + 1 RX), beamforming delay lines, ESP32 config via UART4, 0 errores, 0 underruns.

---

## 1. Hardware

### WeAct Core Board STM32H743VIT6
- MCU: STM32H743VIT6 (ARM Cortex-M7, 480MHz)
- Flash: 2MB interno + 8MB SPI + 8MB QSPI
- RAM: 1MB (DTCMRAM + AXI SRAM + RAM_D1/D2/D3)
- HSE: **25 MHz** (cristal externo)
- Package: LQFP100

### Perifericos usados

| Periferico | Funcion | Pins | AF |
|-----------|---------|------|----|
| SAI1_A | Master TX TDM8 ch0-7 | PE2(MCLK), PE4(FS), PE5(SCK), PE6(SD) | AF6 |
| SAI1_B | Slave TX sync ch8-15 | PE3(SD only) | AF6 |
| SAI2_A | Master TX TDM8 ch16-23 | PD12(FS), PD13(SCK), PD11(SD) | AF10 |
| SAI2_B | Slave TX sync ch24-31 | PE11(SD only) | AF10 |
| SAI3_B | Master RX I2S 2ch | PD8(SCK), PD9(SD), PD10(FS) | AF6 |
| DMA1 | 5 streams audio | Stream0-4 | — |
| UART4 | ESP32 binary protocol | PA0(TX), PA1(RX) | AF8 |
| SPI2 | E-Paper display | PB13(SCK), PB15(MOSI) | AF5 |
| UART7 | Debug serial (ST-Link) | PA8(RX), PA15(TX) | AF11 |
| I2C1 | DAC config (PCM1690) | PB8(SCL), PB9(SDA) | AF4 |
| E-Paper | Display control | PB12(CS), PD4(DC), PD5(RST), PD6(BUSY) | GPIO |

---

## 2. Arquitectura de Audio: DMA Circular con Deferred Processing

### Concepto

Cada SAI block tiene un buffer DMA circular dividido en dos mitades (ping-pong):

```
audio_buffer[7680]:  [====== HALF A (3840) ======][====== HALF B (3840) ======]
                      ^                            ^
                      HalfCplt: fill A              FullCplt: fill B
                      (DMA sending B)               (DMA sending A)
```

El ISR solo setea un flag (~20 ciclos). El main loop hace el trabajo pesado:

```c
// ISR: minimal
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
    if (need_fill_first_half) underrun_count++;  // Deteccion de underrun
    need_fill_first_half = 1;
    __DSB();
}

// Main loop: beamforming
if (need_fill_first_half) {
    need_fill_first_half = 0;
    Fill_Audio_Beamform(&audio_buffer[0], 480, ch_offset);
    SCB_CleanDCache_by_Addr(...);
}
```

### Timing
- Buffer: 480 samples x 8 ch = 3840 int32 por mitad
- 480 / 48000 = 10ms por callback
- 50 half + 50 full = 100 callbacks/s por SAI
- 5 SAI blocks = 500 callbacks/s total

---

## 3. Beamforming Pipeline

### Data Flow

```
SAI3_B RX DMA → audio_buffer_3b_rx[] (interleaved L,R,L,R...)
                       ↓ Process_ADC_Half()
                 delay_line_L[1024] / delay_line_R[1024]  (circular ring buffers)
                       ↓ Fill_Audio_Beamform(buf, samples, ch_offset)
    Per channel: delay_line[write_pos - delay] × gain >> 8 × master >> 8 × mute
                       ↓
    SAI1_A(ch0-7) / SAI1_B(ch8-15) / SAI2_A(ch16-23) / SAI2_B(ch24-31)
```

### Delay Line (Ring Buffer)

```c
#define DELAY_LINE_SIZE  1024   // power of 2 → fast & mask
#define DELAY_LINE_MASK  (DELAY_LINE_SIZE - 1)
#define MAX_DELAY_SAMPLES 64    // ~1.33ms @ 48kHz = ~45.7cm @ 343 m/s

static int32_t delay_line_L[1024];  // beam 1 (ADC left channel)
static int32_t delay_line_R[1024];  // beam 2 (ADC right, future)
static uint32_t delay_write_pos = 0;
```

ADC RX callback → `Process_ADC_Half()` deinterleaves stereo into L/R lines, advances write_pos.

TX fill → `Fill_Audio_Beamform()` reads `delay_line_L[(write_pos - delay - samples + i) & MASK]` per channel.

### Per-Channel Processing (Fixed-Point)

```c
uint16_t delay = beam_config.delays[global_ch];
uint8_t  gain  = beam_config.gains[global_ch];
uint8_t  master = beam_config.master_gain;

int32_t sample = delay_line_L[read_pos];
sample = (sample * gain) >> 8;      // per-channel gain
sample = (sample * master) >> 8;    // master gain
if (muted) sample = 0;
```

### Dual Beam (Futuro)

Arquitectura preparada: `delay_line_R` se llena pero no se lee todavia.
Para activar beam 2:
```c
int32_t sL = delay_line_L[read_pos];
int32_t sR = delay_line_R[read_pos];
sample = ((sL * gain_L) + (sR * gain_R)) >> 9;  // sum/average
```

---

## 4. D-Cache Coherence

Cortex-M7 D-Cache requiere gestion explicita para DMA:

- **TX (CPU escribe → DMA lee):** `SCB_CleanDCache_by_Addr()` despues de llenar el buffer
- **RX (DMA escribe → CPU lee):** `SCB_InvalidateDCache_by_Addr()` antes de leer el buffer

`aligned(32)` en los buffers para alinear a linea de cache.

---

## 5. Configuracion ESP32 → Beamformer

### Protocolo UART4 (115200 8N1)
Trama: `AA 55 [cmd] [len_lo] [len_hi] [payload...] [xor_checksum] 0D 0A`

### ds_config_t (52 bytes, del ESP32)
```c
typedef struct __attribute__((packed)) {
    uint8_t  master_gain;         // 0-255
    uint8_t  mute_global;         // 0/1
    uint16_t delays[16];          // samples per channel (LE)
    uint8_t  gains[16];           // 0-255 per channel
    uint16_t mutes;               // bitfield 16 channels
} ds_config_t;
```

### beam_config_t (local, 32 canales)
```c
typedef struct {
    uint8_t  master_gain;
    uint8_t  mute_global;
    uint16_t delays[32];
    uint8_t  gains[32];
    uint32_t mutes;               // bitfield 32 channels
} beam_config_t;
```

On config receive: 16 ESP32 channels → first 16 slots of beam_config.

---

## 6. Clock Configuration

### System Clock (PLL1) - 480MHz VOS0
```
HSE = 25 MHz
PLL1: M=5, N=192, P=2 → SYSCLK = 480 MHz
HCLK = 240 MHz, APBx = 120 MHz, FLASH_LATENCY_4

VOS0 init: VOS1 → enable SYSCFG → VOS0 (overdrive) → wait VOSRDY
```

### Audio Clock (PLL3) - 12.288 MHz exacto
```
PLL3: M=5, N=68, P=28, FRACN=6660
VCO = 25/5 × (68 + 6660/8192) = 344.065 MHz
MCLK = 344.065 / 28 = 12.288 MHz (error < 0.001%)

Feeds: SAI1, SAI2, SAI3 (via Sai23ClockSelection = PLL3)
```

---

## 7. SAI Sync Mode

SAI1_B y SAI2_B operan en modo sincronizado con sus respectivos bloque A:

- El sub-bloque slave comparte SCK/FS internamente → solo necesita pin SD
- SAI1_B sync: PE3 (SD only). SAI2_B sync: PE11 (SD only)
- **Start order critico**: slave ANTES que master (HAL_SAI_Transmit_DMA slave → master)

---

## 8. DS9 Pin Audit

El PCB DS9 v2.0 tiene **6 errores de mapeo SAI** (ver `ds9_pin_audit.md`):
- SAI1_A: PE7/PE8/PE9 no tienen funcion SAI
- SAI1_B: PE3 conflicto con DAC1_RST
- SAI2_B: PD8/PD9/PD10 = SAI3_B, no SAI2_B
- SAI4_B: PC0/PC1 no tienen SAI4_B SCK/FS en LQFP100

**DS9 plan**: 16ch (SAI1_A + SAI2_A), bodge wire DAC3 para 24ch
**DS10**: Corregido con mapeo correcto (ver tabla en seccion 1)

---

## 9. Lecciones Aprendidas

| Bug | Sintoma | Causa | Fix |
|-----|---------|-------|-----|
| DMA no arranca | HAL_ERROR 0x80+0x40 | Buffer en DTCMRAM | Mover a RAM_D2 `.dma_buffer` |
| SPI cuelga | Hang en EPD_Init | `SPI_DIRECTION_2LINES` sin MISO | `TXONLY` |
| sprintf %f vacio | Numero falta | nano.specs no soporta float | Aritmetica entera |
| Counter overflow | 4294967213/s | uint32 underflow en reset | Resetear last_half/last_full |
| BUSY hang | Loop infinito | Pin flotante HIGH | GPIO_PULLDOWN + epd_available flag |
| 162 underruns | Al inicio | E-paper bloquea 3s | Reset counters post-init |
| 1 underrun espurio | En reporte 60s | UART blocking | Snapshot antes de print |

---

## 10. Archivos del Proyecto

| Archivo | Descripcion |
|---------|-------------|
| `Core/Src/main.c` | Beamforming pipeline, DMA callbacks, main loop |
| `Core/Src/uart_protocol.c` | ESP32 binary protocol parser (state machine) |
| `Core/Inc/uart_protocol.h` | Protocol defines, ds_config_t struct |
| `Core/Src/epaper.c` | Driver SSD1681 e-paper |
| `Core/Src/pcm1690.c` | PCM1690 DAC driver (I2C) |
| `Core/Inc/pin_definitions_ds10.h` | Corrected pin assignments for DS10 |
| `Core/Src/stm32h7xx_hal_msp.c` | MSP init (5x SAI, UART, I2C, SPI) |
| `Core/Src/stm32h7xx_it.c` | IRQ handlers (5x DMA, SAI1, SAI3, UART4) |
| `STM32H743XX_FLASH.ld` | Linker script (.dma_buffer → RAM_D2) |

---

## 11. Referencias

- RM0433: STM32H743 Reference Manual
- AN5543: SAI and I2S on STM32H7
- STM32H743 Datasheet Tables 13-14 (LQFP100 AF mapping)
- SSD1681 Datasheet (e-paper)
- PCM1690 Datasheet (DAC 8ch TDM)
- PCM1802 Datasheet (ADC stereo)
- [WeAct STM32H743 Wiki](https://github.com/WeActStudio/MiniSTM32H7xx)

---

*Firmware de audio beamforming para WeAct STM32H743 — 32ch TX + 2ch RX*
*Pinout DS10, listo para migrar a placa final*
