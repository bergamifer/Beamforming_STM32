# DMA Configuration - WeAct STM32H743 Audio Project

**Source:** RM0433 Reference Manual, Chapter 15-17
**Last Updated:** 2026-02-04
**Status:** Verified working on WeAct board

---

## CRITICAL: STM32H7 Memory Architecture for DMA

### El Error Mas Comun (Lo Vivimos!)

```
❌ DTCMRAM (0x20000000) - DMA1/DMA2 NO PUEDEN ACCEDER
✅ RAM_D2  (0x30000000) - DMA1/DMA2 SI pueden acceder
✅ RAM_D3  (0x38000000) - BDMA puede acceder
```

**Sintoma:** `HAL_SAI_Transmit_DMA()` retorna `HAL_ERROR`, error code `0x80` (DMA) + `0x40` (TIMEOUT)

**Solucion:**
```c
// En main.c - usar seccion especial
static int32_t audio_buffer[SIZE] __attribute__((section(".dma_buffer"), aligned(4)));
```

```ld
// En linker script (.ld)
.dma_buffer (NOLOAD) :
{
  . = ALIGN(4);
  *(.dma_buffer)
  . = ALIGN(4);
} >RAM_D2
```

### Mapa de Memoria STM32H7

```
┌─────────────────────────────────────────────────────────┐
│                    D1 Domain                             │
│  ┌──────────┐  ┌──────────┐                             │
│  │  DTCMRAM │  │ AXI SRAM │                             │
│  │0x20000000│  │0x24000000│                             │
│  │   128K   │  │   512K   │                             │
│  │ CPU ONLY │  │ CPU+MDMA │  ← Variables por defecto    │
│  └──────────┘  └──────────┘    van aca (NO DMA!)        │
├─────────────────────────────────────────────────────────┤
│                    D2 Domain                             │
│  ┌──────────────────────────────────────────┐           │
│  │  RAM_D2 (SRAM1+2+3)                      │           │
│  │  0x30000000 - 288K                       │           │
│  │  ✅ DMA1/DMA2 pueden acceder             │           │
│  │  ← Poner buffers DMA aca!                │           │
│  └──────────────────────────────────────────┘           │
├─────────────────────────────────────────────────────────┤
│                    D3 Domain                             │
│  ┌──────────┐                                           │
│  │  RAM_D3  │  ← BDMA (para SAI4, LPUART, etc)         │
│  │0x38000000│                                           │
│  │   64K    │                                           │
│  └──────────┘                                           │
└─────────────────────────────────────────────────────────┘
```

---

## Configuracion Actual: WeAct Test Board

### Hardware
- **MCU:** STM32H743VIT6 (WeAct Mini)
- **Audio:** SAI1_A Master TX, I2S stereo
- **Clocks:** MCLK 12.288 MHz, BCK 3.072 MHz, LRCK 48 kHz

### DMA Actual

| Periferico | Controlador | Stream | DMAMUX | Request | Direccion | Estado |
|------------|-------------|--------|--------|---------|-----------|--------|
| SAI1_A TX  | DMA1        | Stream 0 | DMAMUX1 | 87 | M2P | ✅ Funcionando |

### Configuracion en CubeMX/Codigo

```c
// stm32h7xx_hal_msp.c - HAL_SAI_MspInit()
hdma_sai1_a.Instance = DMA1_Stream0;
hdma_sai1_a.Init.Request = DMA_REQUEST_SAI1_A;      // DMAMUX1 request 87
hdma_sai1_a.Init.Direction = DMA_MEMORY_TO_PERIPH;
hdma_sai1_a.Init.PeriphInc = DMA_PINC_DISABLE;
hdma_sai1_a.Init.MemInc = DMA_MINC_ENABLE;
hdma_sai1_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;  // 32-bit
hdma_sai1_a.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;     // 32-bit
hdma_sai1_a.Init.Mode = DMA_CIRCULAR;               // Double buffering
hdma_sai1_a.Init.Priority = DMA_PRIORITY_HIGH;
hdma_sai1_a.Init.FIFOMode = DMA_FIFOMODE_DISABLE;   // Direct mode
```

### Buffer Audio (en RAM_D2)

```c
// main.c
#define AUDIO_BUFFER_SAMPLES 480      // 10ms @ 48kHz
#define AUDIO_CHANNELS       2        // Stereo
#define AUDIO_BUFFER_SIZE    (AUDIO_BUFFER_SAMPLES * AUDIO_CHANNELS)  // 960

// DEBE estar en RAM_D2!
static int32_t audio_buffer[AUDIO_BUFFER_SIZE * 2]
    __attribute__((section(".dma_buffer"), aligned(4)));
// Total: 960 × 2 × 4 = 7680 bytes
```

---

## Configuracion Futura: DS9 Board (32+2 canales)

### Objetivo
- **Salida:** 32 canales @ 48kHz, 16-bit (2× DAC de 16 canales)
- **Entrada:** 2 canales @ 48kHz, 16-bit (1× ADC stereo)

### Opcion A: SAI1 TDM32 (32 slots en un bloque)

```
SAI1_A: 32 slots × 16 bits = 512 bits/frame
BCK = 48,000 × 512 = 24.576 MHz ✓
```

| Periferico | Controlador | Stream | Request | Canales | Bandwidth |
|------------|-------------|--------|---------|---------|-----------|
| SAI1_A TX  | DMA1 | Stream 0 | 87 | 32 | 3.07 MB/s |
| SAI2_A RX  | DMA1 | Stream 1 | 89 | 2 | 0.19 MB/s |
| **Total** | | | | **34** | **3.26 MB/s** |

### Opcion B: SAI1_A + SAI1_B (16+16 slots)

```
SAI1_A: 16 slots → DAC 0-15
SAI1_B: 16 slots → DAC 16-31 (sync con A)
BCK = 48,000 × 256 = 12.288 MHz ✓
```

| Periferico | Controlador | Stream | Request | Canales | Bandwidth |
|------------|-------------|--------|---------|---------|-----------|
| SAI1_A TX  | DMA1 | Stream 0 | 87 | 16 | 1.54 MB/s |
| SAI1_B TX  | DMA1 | Stream 5 | 88 | 16 | 1.54 MB/s |
| SAI2_A RX  | DMA1 | Stream 1 | 89 | 2 | 0.19 MB/s |
| **Total** | | | | **34** | **3.27 MB/s** |

### Buffers para DS9 (10ms latencia)

```c
// Salida 32 canales: 32 × 480 × 2 bytes × 2 (double) = 61,440 bytes
#define OUT_BUFFER_SIZE  (32 * 480 * 2)
static int16_t out_buffer[OUT_BUFFER_SIZE * 2]
    __attribute__((section(".dma_buffer"), aligned(4)));

// Entrada 2 canales: 2 × 480 × 2 bytes × 2 (double) = 3,840 bytes
#define IN_BUFFER_SIZE   (2 * 480 * 2)
static int16_t in_buffer[IN_BUFFER_SIZE * 2]
    __attribute__((section(".dma_buffer"), aligned(4)));

// Total: ~65 KB en RAM_D2 (de 288 KB disponibles) ✓
```

---

## DMAMUX Request IDs (RM0433 Table 122)

### DMAMUX1 (para DMA1/DMA2)

| Request ID | Periferico | Notas |
|------------|------------|-------|
| 87 | SAI1_A | TX/RX |
| 88 | SAI1_B | TX/RX |
| 89 | SAI2_A | TX/RX |
| 90 | SAI2_B | TX/RX |
| 113 | SAI3_A | TX/RX |
| 114 | SAI3_B | TX/RX |

### DMAMUX2 (para BDMA - D3 domain)

| Request ID | Periferico | Notas |
|------------|------------|-------|
| 15 | SAI4_A | TX/RX |
| 16 | SAI4_B | TX/RX |

**Nota:** SAI4 esta en D3 domain y DEBE usar BDMA, no puede usar DMA1/DMA2.

---

## Modo Circular y Callbacks

### Como Funciona

```
     Buffer en RAM_D2
    ┌─────────────────────────────────────┐
    │  First Half    │   Second Half      │
    │  (480 samples) │   (480 samples)    │
    └────────┬───────┴────────┬───────────┘
             │                │
             ▼                ▼
      HAL_SAI_TxHalfCpltCallback   HAL_SAI_TxCpltCallback
      (refill 1st half)            (refill 2nd half)
             │                      │
             └────────┬─────────────┘
                      ▼
               DMA vuelve al inicio
               automaticamente (circular)
```

### Callbacks en Codigo

```c
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
    // DMA esta transmitiendo 2da mitad
    // Rellenar 1ra mitad del buffer
    Fill_Audio_Buffer(&audio_buffer[0], AUDIO_BUFFER_SAMPLES);
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
    // DMA esta transmitiendo 1ra mitad
    // Rellenar 2da mitad del buffer
    Fill_Audio_Buffer(&audio_buffer[AUDIO_BUFFER_SIZE], AUDIO_BUFFER_SAMPLES);
}
```

---

## Capacidad del Sistema

| Recurso | Capacidad Total | Uso Audio (34ch) | % Utilizado |
|---------|-----------------|------------------|-------------|
| DMA bandwidth | ~200 MB/s | 3.3 MB/s | 1.6% |
| DMA streams | 16 | 3 | 19% |
| RAM_D2 | 288 KB | 65 KB | 23% |
| SAI BCK max | ~50 MHz | 24.6 MHz | 49% |

**Conclusion:** El sistema tiene capacidad de sobra para 32+2 canales.

---

## Checklist de Configuracion DMA

- [ ] Habilitar clock DMA: `__HAL_RCC_DMA1_CLK_ENABLE()`
- [ ] Buffers en RAM_D2: `__attribute__((section(".dma_buffer")))`
- [ ] Linker script con seccion `.dma_buffer` en `>RAM_D2`
- [ ] NVIC habilitado para DMA stream interrupt
- [ ] Modo circular para audio continuo
- [ ] Prioridad HIGH para audio real-time
- [ ] Data alignment 32-bit para SAI 24-bit

---

## Referencias

- **RM0433** STM32H742/H743/H753/H750 Reference Manual
  - Chapter 15: DMA controllers
  - Chapter 16: BDMA controller
  - Chapter 17: DMAMUX request multiplexer
  - Table 122: DMAMUX1 request assignments
  - Table 126: DMAMUX2 request assignments
