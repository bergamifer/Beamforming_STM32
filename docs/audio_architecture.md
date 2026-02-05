# DS Beamforming - Arquitectura de Audio v2.0

**Estado:** Actualizado para 32 canales (4× PCM1690)
**Fecha:** Febrero 2026
**Referencia:** DS_Hardware_Map_v2.md (DS.d356 netlist)

---

## 1. Resumen del Sistema

```
                        ┌──────────────────────────────────────────────────────────────┐
                        │                      STM32H743VIT6                           │
                        │                                                              │
   Micrófono ──────────►│ PB11 (SAI4_SD_B) ◄── PCM1802 U1 (ADC)                      │
                        │ PC1  (SAI4_SCK_B)──► │                                      │
                        │ PC0  (SAI4_FS_B) ──► │                                      │
                        │                                                              │
                        │ PE2 (SAI1_MCLK) ─── R21 (33Ω) ───┬──────────────────┐      │
                        │                                    │  Clock Bus        │      │
                        │                                    │  12.288 MHz       │      │
                        │                                    ▼         ▼    ▼    ▼      │
                        │ SAI1_A: PE7/PE8/PE9 ──►  PCM1690 U3 (DAC0)  ch 0-7  │      │──► 8 Speakers
                        │ SAI1_B: PE4/PE5/PE6 ──►  PCM1690 U4 (DAC1)  ch 8-15 │      │──► 8 Speakers
                        │ SAI2_A: PD13/PD12/PD11 ► PCM1690 U2 (DAC2)  ch 16-23│      │──► 8 Speakers
                        │ SAI2_B: PD10/PD9/PD8 ►   PCM1690 U8 (DAC3)  ch 24-31│      │──► 8 Speakers
                        │                                                       │      │
                        │ I2C1 (PB8/PB9) ──────── DAC config (4× PCM1690)       │      │
                        │ I2C3 (PB6/PB7) ──────── EEPROM CAT24C256 (calibración) │     │
                        │ UART4 (PA0/PA1) ─────── ESP32-C3 (WiFi control)        │     │
                        └──────────────────────────────────────────────────────────┘
```

---

## 2. Componentes de Audio

| Componente | Ref | Función | Bus SAI | I2C Addr (8-bit) | Canales |
|------------|-----|---------|---------|-----------------|---------|
| PCM1802 | U1 | ADC Stereo | SAI4_B | - (GPIO ctrl) | 2 entrada |
| PCM1690 | U3 | DAC 8-ch | SAI1_A | 0x94 | 0-7 |
| PCM1690 | U4 | DAC 8-ch | SAI1_B | 0x96 | 8-15 |
| PCM1690 | U2 | DAC 8-ch | SAI2_A | 0x98 | 16-23 |
| PCM1690 | U8 | DAC 8-ch | SAI2_B | 0x9A | 24-31 |
| **Total** | | | | | **2 in / 32 out** |

---

## 3. Sincronización de Clocks - CRÍTICO

### 3.1 Arquitectura de Clock Distribution

```
                         PE2 (SAI1_MCLK)
                               │
                          R21 (33Ω)
                               │
                     12.288 MHz (256 × fS)
                               │
          ┌────────┬───────────┼───────────┬────────┐
          │        │           │           │        │
          ▼        ▼           ▼           ▼        ▼
     ┌────────┐┌────────┐┌────────┐┌────────┐┌────────┐
     │PCM1802 ││PCM1690 ││PCM1690 ││PCM1690 ││PCM1690 │
     │  U1    ││  U3    ││  U4    ││  U2    ││  U8    │
     │  ADC   ││ DAC0   ││ DAC1   ││ DAC2   ││ DAC3   │
     └────────┘└────────┘└────────┘└────────┘└────────┘
       2ch in   ch 0-7    ch 8-15  ch 16-23  ch 24-31
```

**PUNTO CLAVE:** Los 5 chips comparten el mismo MCLK derivado de PE2. Esto garantiza:
- Cero drift entre ADC y los 4 DACs
- Muestras perfectamente alineadas en tiempo
- Latencia determinística para beamforming

### 3.2 Frecuencias de Clock

| Señal | Frecuencia | Relación | Origen |
|-------|------------|----------|--------|
| MCLK | 12.288 MHz | 256 × fS | PE2 (SAI1_MCLK) → R21 → Bus |
| BCK (TDM8) | 3.072 MHz | 64 × fS | SAI_SCK por bloque |
| FS (Frame Sync) | 48 kHz | fS | SAI_FS por bloque |

### 3.3 Configuración PLL3 para 12.288 MHz

```
HSE = 25 MHz

PLL3:
  DIVM3 = 5        →  5 MHz referencia
  DIVN3 = 68       → 340 MHz VCO
  FRACN3 = ajustar →  refinamiento fino
  DIVP3 = 28       →  12.143 MHz base

  Con FRACN correctamente calculado → 12.288 MHz exactos

Verificación:
  12.288 MHz / 48000 Hz = 256  ✅ (ratio MCLK/fs)
  12.288 MHz / 3072000 Hz = 4  ✅ (ratio MCLK/BCK)
```

---

## 4. Mapeo SAI → DAC (TDM8)

### 4.1 SAI1 - DAC0 y DAC1 (Canales 0-15)

```
SAI1_MCLK ──► PE2 (Master Clock output)

SAI1_A (DAC0 - U3, canales 0-7):
  PE7  ──► FS  (Frame Sync)
  PE8  ──► SCK (Bit Clock)
  PE9  ──► SD  (Serial Data)

SAI1_B (DAC1 - U4, canales 8-15):
  PE4  ──► FS  (Frame Sync)
  PE5  ──► SCK (Bit Clock)
  PE6  ──► SD  (Serial Data)

Frame structure (256 bits):
  ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
  │ Ch 0 │ Ch 1 │ Ch 2 │ Ch 3 │ Ch 4 │ Ch 5 │ Ch 6 │ Ch 7 │
  │32 bits│32 bits│32 bits│32 bits│32 bits│32 bits│32 bits│32 bits│
  └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
  ▲ FS pulse
```

### 4.2 SAI2 - DAC2 y DAC3 (Canales 16-31)

```
SAI2_A (DAC2 - U2, canales 16-23):
  PD13 ──► FS  (Frame Sync)
  PD12 ──► SCK (Bit Clock)
  PD11 ──► SD  (Serial Data)

SAI2_B (DAC3 - U8, canales 24-31):
  PD10 ──► FS  (Frame Sync)
  PD9  ──► SCK (Bit Clock)
  PD8  ──► SD  (Serial Data)

SAI2 sincronizado con SAI1 (mismo MCLK source)
```

### 4.3 SAI4 - ADC (Entrada)

```
SAI4_B (ADC - U1, 2 canales stereo):
  PC0  ──► FS  (Frame Sync → LRCK del ADC)
  PC1  ──► SCK (Bit Clock → BCK del ADC)
  PB11 ◄── SD  (Serial Data ← DOUT del ADC)

⚠️ SAI4 está en dominio D3 → DMA debe ser BDMA
```

### 4.4 Resumen Completo

| SAI Block | Pines (FS / SCK / SD) | DAC | Ref | Canales | DMA |
|-----------|----------------------|-----|-----|---------|-----|
| SAI1_A | PE7 / PE8 / PE9 | DAC0 | U3 | 0-7 | DMA1 |
| SAI1_B | PE4 / PE5 / PE6 | DAC1 | U4 | 8-15 | DMA1 |
| SAI2_A | PD13 / PD12 / PD11 | DAC2 | U2 | 16-23 | DMA1 |
| SAI2_B | PD10 / PD9 / PD8 | DAC3 | U8 | 24-31 | DMA1 |
| SAI4_B | PC0 / PC1 / PB11 | ADC | U1 | input 2ch | **BDMA** |

---

## 5. DMA y BDMA Configuration

### 5.1 DMA1 - Audio Output (Dominio D2)

| Stream | SAI Block | DMAMUX Request | Dirección | DAC |
|--------|-----------|----------------|-----------|-----|
| DMA1 Stream 0 | SAI1_A | 87 | Mem → Per | DAC0 (U3) |
| DMA1 Stream 1 | SAI1_B | 88 | Mem → Per | DAC1 (U4) |
| DMA1 Stream 2 | SAI2_A | 89 | Mem → Per | DAC2 (U2) |
| DMA1 Stream 3 | SAI2_B | 90 | Mem → Per | DAC3 (U8) |

### 5.2 BDMA - Audio Input (Dominio D3)

| Channel | SAI Block | DMAMUX2 Request | Dirección | ADC |
|---------|-----------|-----------------|-----------|-----|
| BDMA Ch 7 | SAI4_B | 16 | Per → Mem | ADC (U1) |

### 5.3 Buffer Structure (TDM8)

```c
#define SAMPLES_PER_FRAME   480         // 10ms @ 48kHz
#define CHANNELS_PER_DAC    8           // TDM8
#define DAC_COUNT           4
#define ADC_CHANNELS        2

// Output buffers (double buffer para cada DAC)
uint32_t dac0_buffer[2][SAMPLES_PER_FRAME * CHANNELS_PER_DAC];  // DAC0: ch 0-7
uint32_t dac1_buffer[2][SAMPLES_PER_FRAME * CHANNELS_PER_DAC];  // DAC1: ch 8-15
uint32_t dac2_buffer[2][SAMPLES_PER_FRAME * CHANNELS_PER_DAC];  // DAC2: ch 16-23
uint32_t dac3_buffer[2][SAMPLES_PER_FRAME * CHANNELS_PER_DAC];  // DAC3: ch 24-31

// Input buffer (double buffer ADC)
uint32_t adc_buffer[2][SAMPLES_PER_FRAME * ADC_CHANNELS];       // ADC: 2ch stereo

// Total RAM audio buffers:
//   Output: 4 DACs × 2 buffers × 480 × 8 × 4 bytes = 122.88 KB
//   Input:  1 × 2 buffers × 480 × 2 × 4 bytes       =   7.68 KB
//   Total:  ~131 KB (disponible en STM32H7: 1 MB SRAM)
```

---

## 6. Configuración de Hardware v2.0

### 6.1 PCM1802 (ADC) - U1

| Función | Pin STM32 | Pin PCM1802 | Estado por defecto |
|---------|-----------|-------------|-------------------|
| PDWN | PA7 | 7 | LOW (power down) → activar en power-up |
| OSR | PA5 | 16 | HIGH (128× oversampling) |
| BYPASS | PA6 | 8 | LOW (HPF activo) |
| FSYNC | PA4 | 9 | LOW |
| FMT0 | PC7 | 17 | LOW → I2S con FMT1=LOW |
| FMT1 | PC6 | 18 | LOW → I2S |
| MODE0 | PD14 | 19 | LOW → Slave con MODE1=LOW |
| MODE1 | PD15 | 20 | LOW → Slave |

### 6.2 PCM1690 (DACs) - U3, U4, U2, U8

| DAC | Ref | RST Pin | AMUTEI | AMUTEO | I2C Addr | SAI |
|-----|-----|---------|--------|--------|----------|-----|
| DAC0 | U3 | PE10 | PE11 | PE12 | 0x94 | SAI1_A |
| DAC1 | U4 | PE3 | PE1 | PE0 | 0x96 | SAI1_B |
| DAC2 | U2 | PC10 | PC11 | PC12 | 0x98 | SAI2_A |
| DAC3 | U8 | PB15 | PB14 | PB13 | 0x9A | SAI2_B |

### 6.3 I2C Bus Assignment

```
I2C1 (PB8/PB9) - Bus DACs:
  ├── 0x94  PCM1690 U3 (DAC0)  ADR1=0, ADR0=0
  ├── 0x96  PCM1690 U4 (DAC1)  ADR1=0, ADR0=1
  ├── 0x98  PCM1690 U2 (DAC2)  ADR1=1, ADR0=0
  └── 0x9A  PCM1690 U8 (DAC3)  ADR1=1, ADR0=1

I2C3 (PB6/PB7) - Bus EEPROM:
  └── 0xA0  CAT24C256 U6        A0-A2=GND (7-bit: 0x50)
            WP: PB5 (HIGH=protecto)
```

---

## 7. Flujo de Audio para Beamforming (32 canales)

```
┌──────────┐   ┌──────────┐   ┌──────────────────────┐   ┌──────────┐   ┌───────────┐
│Micrófono │──►│ PCM1802  │──►│      STM32H743       │──►│ 4×PCM1690│──►│ 32 Speakers│
│          │   │   ADC    │   │                      │   │   DACs   │   │            │
└──────────┘   └──────────┘   │  Por cada frame:     │   └──────────┘   └───────────┘
                              │  ┌──────────────┐    │
                              │  │ Read 2ch ADC │    │
                              │  └──────┬───────┘    │
                              │         ▼            │
                              │  ┌──────────────┐    │
                              │  │ Beamforming  │    │
                              │  │ - Delay/ch   │    │
                              │  │ - Gain/ch    │    │
                              │  └──────┬───────┘    │
                              │         ▼            │
                              │  ┌──────────────┐    │
                              │  │ Fill 32ch out│    │
                              │  │ - DAC0: 0-7  │    │
                              │  │ - DAC1: 8-15 │    │
                              │  │ - DAC2: 16-23│    │
                              │  │ - DAC3: 24-31│    │
                              │  └──────────────┘    │
                              └──────────────────────┘
```

### 7.1 Latencia del Sistema

| Etapa | Latencia | Notas |
|-------|----------|-------|
| PCM1802 decimation filter | 17.4 / fS | ~362 µs @ 48kHz |
| STM32 DMA input buffer | N / fS | Configurable (480 samples = 10ms) |
| STM32 procesamiento | < 1 / fS | Beamforming en callback |
| STM32 DMA output buffer | N / fS | Misma latencia que input |
| PCM1690 interpolation | ~1 / fS | ~21 µs @ 48kHz |
| **Total mínimo** | ~400 µs + 2×buffer | Sin buffer: ~400µs, con 480 samples: ~20.4ms |

### 7.2 Requisitos de Timing para Beamforming

Para audio direccional, el delay máximo entre canales depende de:
- Distancia entre speakers
- Velocidad del sonido (~343 m/s)

Ejemplos:
- Speakers separados 5 cm → Delay máximo = 146 µs = **7 samples @ 48kHz**
- Speakers separados 10 cm → Delay máximo = 292 µs = **14 samples @ 48kHz**
- Speakers separados 50 cm → Delay máximo = 1.458 ms = **70 samples @ 48kHz**

---

## 8. Secuencia de Inicialización v2.0

```c
void Audio_Init(void)
{
    // 1. Enable 3.3V analog
    PWR_Enable_3V3A();
    HAL_Delay(10);

    // 2. Hold all 4 DACs in reset
    HAL_GPIO_WritePin(DAC0_RST_PORT, DAC0_RST_PIN, GPIO_PIN_RESET);  // U3
    HAL_GPIO_WritePin(DAC1_RST_PORT, DAC1_RST_PIN, GPIO_PIN_RESET);  // U4
    HAL_GPIO_WritePin(DAC2_RST_PORT, DAC2_RST_PIN, GPIO_PIN_RESET);  // U2
    HAL_GPIO_WritePin(DAC3_RST_PORT, DAC3_RST_PIN, GPIO_PIN_RESET);  // U8

    // 3. ADC power down
    ADC_PowerOff();

    // 4. Start MCLK via SAI1
    HAL_SAI_Init(&hsai_BlockA1);  // PE2 → 12.288 MHz
    HAL_Delay(1);

    // 5. Release DACs sequentially (1ms entre cada uno)
    HAL_GPIO_WritePin(DAC0_RST_PORT, DAC0_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(DAC1_RST_PORT, DAC1_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(DAC2_RST_PORT, DAC2_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(DAC3_RST_PORT, DAC3_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    // 6. Configure DACs via I2C1 (TDM8 format)
    PCM1690_Init(&hdac0);  // U3 @ 0x94
    PCM1690_Init(&hdac1);  // U4 @ 0x96
    PCM1690_Init(&hdac2);  // U2 @ 0x98
    PCM1690_Init(&hdac3);  // U8 @ 0x9A

    // 7. Configure ADC format (I2S, Slave, 128x OSR)
    ADC_SetFormat(ADC_FORMAT_I2S);
    ADC_SetMode(ADC_MODE_SLAVE);
    ADC_SetOSR(1);          // 128x oversampling
    ADC_SetHPFBypass(0);    // HPF active

    // 8. Power on ADC (requires 100ms startup)
    ADC_PowerOn();
    HAL_Delay(100);

    // 9. Load calibration from EEPROM (I2C3)
    EEPROM_WriteProtect_Disable();
    EEPROM_Calibration_t cal;
    EEPROM_Read_Calibration(&hi2c3, &cal);
    Apply_Calibration(&cal);
    EEPROM_WriteProtect_Enable();

    // 10. Start all DMA transfers
    HAL_SAI_Receive_DMA(&hsai_BlockB4, adc_buffer[0], ADC_BUFFER_SIZE);
    HAL_SAI_Transmit_DMA(&hsai_BlockA1, dac0_buffer[0], DAC_BUFFER_SIZE);
    HAL_SAI_Transmit_DMA(&hsai_BlockB1, dac1_buffer[0], DAC_BUFFER_SIZE);
    HAL_SAI_Transmit_DMA(&hsai_BlockA2, dac2_buffer[0], DAC_BUFFER_SIZE);
    HAL_SAI_Transmit_DMA(&hsai_BlockB2, dac3_buffer[0], DAC_BUFFER_SIZE);
}
```

---

## 9. Checklist de Verificación v2.0

### Hardware ✅
- [x] MCLK compartido entre ADC y los 4 DACs
- [x] PCM1802 en modo Slave
- [x] PCM1802 en formato I2S 24-bit
- [x] 4× PCM1690 conectados a I2C1
- [x] EEPROM conectado a I2C3
- [x] Bypass capacitors en VREF
- [x] Alimentación 5V/3.3V correcta
- [x] ESP32-C3 conectado via UART4
- [x] Test Points en PE13, PE14, PA8

### Firmware (Pendiente)
- [ ] PLL3 configurado para 12.288 MHz exactos
- [ ] SAI1 MCLK habilitado en PE2
- [ ] SAI1_A y SAI1_B configurados (TDM8)
- [ ] SAI2_A y SAI2_B configurados (TDM8)
- [ ] SAI4_B configurado para recepción ADC
- [ ] DMA1 configurado para 4 DACs
- [ ] BDMA configurado para ADC
- [ ] I2C1 inicializa los 4 PCM1690
- [ ] I2C3 funciona con EEPROM
- [ ] Secuencia power-up implementada
- [ ] Procesamiento beamforming 32 canales
- [ ] Calibración automática (ver Sección 10)

---

## 10. Desafío Avanzado: Calibración Automática por Micrófono

### 10.1 Concepto

El beamforming requiere que todos los canales lleguen **en fase** al punto de escucha. En la realidad, cada canal tiene una demora total diferente por:

- Latencia electrónica del DAC (varía por chip)
- Longitud del cable hasta el speaker
- Distancia física del speaker al punto de escucha
- Variación de tolerancia entre amplificadores

La idea es usar el **micrófono propio del sistema** (conectado al ADC PCM1802) para medir automáticamente la demora de cada canal y pre-compensarla.

### 10.2 Arquitectura del Sistema de Calibración

```
                    Fase 1: Emisión secuencial
                    ─────────────────────────
  STM32 ──► DAC ch N ──► Amplificador ──► Speaker N ──► Aire
                                                         │
                                                         ▼
                                              ┌─── Punto de escucha ───┐
                                                         │
                                                         ▼
                                          Micrófono ──► ADC ──► STM32
                                                                  │
                                                    Fase 2: Medir delay
                                                    ─────────────────────

  Para cada canal (0 → 31):
    1. Emitir impulso conocido por canal N
    2. Capturar señal del micrófono
    3. Calcular cross-correlation entre impulso y captura
    4. El lag máximo = demora total del canal N
    5. Guardar delay[N] en EEPROM
```

### 10.3 Procedimiento de Calibración

```
┌─────────────────────────────────────────────────────────────────┐
│                  CALIBRACIÓN AUTOMÁTICA                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  PASO 1: Preparación                                            │
│  ├── Mute todos los canales                                     │
│  ├── Posicionar micrófono en el punto de escucha                │
│  └── Aguardar estabilización del sistema                        │
│                                                                 │
│  PASO 2: Emisión secuencial (loop por cada canal)               │
│  ├── Channel 0:                                                 │
│  │   ├── Unmute ch 0, mute resto                                │
│  │   ├── Emitir chirp/impulso conocido                          │
│  │   ├── Capturar N samples del ADC                             │
│  │   └── Mute ch 0                                              │
│  ├── Channel 1: (mismo proceso)                                 │
│  │   └── ...                                                    │
│  └── Channel 31: (mismo proceso)                               │
│                                                                 │
│  PASO 3: Procesamiento offline                                  │
│  ├── Para cada canal: cross-correlación señal emitida/capturada │
│  ├── Encontrar lag máximo → delay_raw[N]                        │
│  ├── delay_compensado[N] = delay_raw[max] - delay_raw[N]        │
│  └── El canal más lejano tiene delay_compensado = 0             │
│                                                                 │
│  PASO 4: Persistencia                                           │
│  ├── Guardar delays en EEPROM (CAT24C256)                       │
│  └── Aplicar calibración en runtime                             │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 10.4 Señal de Calibración: Chirp vs Impulso

```
Opción A: Impulso (simple, rápido)
  ┌─┐
  │ │
──┘ └──────────────────
  1 muestra a amplitud máxima
  Pros: Simple de implementar
  Cons: Bajo SNR si hay ruido ambiente

Opción B: Chirp sweeping (robusto)  ← RECOMENDADO
  /\/\/\/\/\
 /          \
─            ────────────
  Tono que sube de frecuencia linealmente
  Ej: 1kHz → 10kHz en 10ms
  Pros: Alta SNR, inmune a ruido
  Cons: Requiere cross-correlación más compleja

Opción C: Secuencia PN (máxima longitud)  ← MUY ROBUSTO
  ┌─┐ ┌───┐ ┌─┐ ┌─┐   ┌───┐
  │ │ │   │ │ │ │ │   │   │
──┘ └─┘   └─┘ └─┘ └───┘   └──
  Pseudorandom binary sequence
  Pros: Excelente en autocorrelación
  Cons: Implementación más compleja
```

### 10.5 Algoritmo de Medición de Delay

```c
// Pseudocódigo - Calibración automática
typedef struct {
    int32_t delay_samples[32];      // Demora medida por canal
    int32_t delay_compensated[32];  // Compensación a aplicar
    float   gain_cal[32];           // Calibración de ganancia
    uint8_t valid;                  // 1 si la calibración es válida
    uint16_t crc;                   // CRC para verificación
} Calibration_t;

HAL_StatusTypeDef Calibrate_Channel(uint8_t channel, int32_t *measured_delay)
{
    // 1. Mute all, unmute solo el canal objetivo
    DAC_Set_Mute(1);                          // Mute all
    DAC_Set_Mute_Single(channel / 8, 0);      // Unmute DAC correspondiente
    // (agregar control por canal individual dentro del DAC vía I2C)

    // 2. Preparar señal de referencia (chirp)
    int32_t ref_signal[CHIRP_LENGTH];
    Generate_Chirp(ref_signal, CHIRP_LENGTH, 1000, 10000, 48000);

    // 3. Escribir señal al buffer del canal correcto
    memset(dac_buffer_current, 0, sizeof(dac_buffer_current));
    Copy_To_Channel(dac_buffer_current, channel % 8, ref_signal, CHIRP_LENGTH);

    // 4. Trigear DMA y capturar desde ADC simultáneamente
    Start_Calibration_Capture(adc_capture_buffer, CAPTURE_LENGTH);
    HAL_Delay(CAPTURE_LENGTH / 48 + 10);  // Esperar captura completa

    // 5. Cross-correlación
    *measured_delay = Cross_Correlate(ref_signal, CHIRP_LENGTH,
                                       adc_capture_buffer, CAPTURE_LENGTH);

    // 6. Mute channel
    DAC_Set_Mute(1);

    return HAL_OK;
}

HAL_StatusTypeDef Run_Full_Calibration(Calibration_t *cal)
{
    int32_t max_delay = 0;

    // Calibrar los 32 canales secuencialmente
    for (uint8_t ch = 0; ch < 32; ch++) {
        Calibrate_Channel(ch, &cal->delay_samples[ch]);
        if (cal->delay_samples[ch] > max_delay)
            max_delay = cal->delay_samples[ch];
    }

    // Calcular compensación: el canal más lejano tiene comp = 0
    for (uint8_t ch = 0; ch < 32; ch++) {
        cal->delay_compensated[ch] = max_delay - cal->delay_samples[ch];
    }

    cal->valid = 1;
    cal->crc = Calculate_CRC16((uint8_t*)cal, sizeof(Calibration_t) - 2);

    // Guardar en EEPROM
    EEPROM_WriteProtect_Disable();
    EEPROM_Write_Calibration(&hi2c3, cal);
    EEPROM_WriteProtect_Enable();

    return HAL_OK;
}
```

### 10.6 Aplicación en Runtime

```
Una vez calibrado, en el procesamiento por frame:

Para cada frame de 480 samples:
  1. Leer 2ch del ADC
  2. Para cada canal de salida (0-31):
     a. Tomar la muestra de entrada
     b. Agregar delay_compensado[N] samples (buffer circular)
     c. Aplicar gain_cal[N]
     d. Escribir al slot correcto del DAC

Resultado: todos los canales "salen" en fase
           al punto donde estaba el micrófono
           → foco automático en ese punto

Buffer de delay necesario:
  max_delay_samples × sizeof(int32_t) × 32 canales
  Ejemplo: 100 samples delay máximo
  = 100 × 4 × 32 = 12.8 KB
```

### 10.7 Estudio de Viabilidad

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| Tiempo por canal | ~50ms | Chirp 10ms + captura + procesamiento |
| Tiempo total calibración | ~1.6s | 32 canales × 50ms |
| Precisión delay | ±1 sample (21µs) | Con chirp y cross-correlación |
| Almacenamiento | 226 bytes | Fit en EEPROM (32KB disponibles) |
| RAM necesaria | ~15 KB extra | Buffers de captura y correlación |
| Frecuencia recomendada | Cada arranque o manual | Antes de cada presentación |

### 10.8 Extensiones Futuras

Una vez que el sistema de calibración básico funciona, se pueden considerar:

1. **Calibración de ganancia:** Emitir señal conocida, medir amplitud en el micrófono por canal. Nivelar todos a la misma amplitud → compensación de ganancia automática.

2. **Multi-punto:** Calibrar con varios micrófonos en diferentes posiciones. Crear un mapa de demoras en el espacio. Permitir elegir punto de foco en runtime sin recalibrar.

3. **Calibración adaptativa:** En runtime, usar el micrófono para monitorear continuamente la fase relativa entre canales. Si detecta drift, ajustar automáticamente.

4. **Beamforming steerable:** Dar al usuario coordenadas (x, y) del punto de foco deseado. El sistema calcula los delays necesarios combinando la calibración base + geometría.

---

## 11. referencias

- [DS_Hardware_Map_v2.md](DS_Hardware_Map_v2.md) - Pinout completo y netlist verificado
- [pcm1802_registers.md](pcm1802_registers.md) - Detalles del ADC
- [pcm1690_ref.md](pcm1690_ref.md) - Detalles de los DACs
- [gpio_control.md](gpio_control.md) - Control GPIO detallado
- DS.d356 - Netlist del PCB (verificación de hardware)

---

*Documento de arquitectura de audio - DS Beamforming v2.0*
*32 canales de salida / 2 canales de entrada*
*Actualizado: Febrero 2026*
