# Comandos y Referencias - WeAct Audio Test

## Checklist al iniciar sesion

- [ ] Conectar ST-Link (SWD + UART)
- [ ] Abrir minicom en terminal separada
- [ ] Si pregunta modo, elegir **Debug**

---

## Build y Flash

```bash
cd /Users/bergamifer/STM32CubeIDE/WeAct_test_audio/firmware

# Solo compilar
make -j4

# Compilar + Flash via ST-Link (recomendado)
make flash-stlink

# Compilar + Flash via DFU (requiere modo bootloader)
make flash

# Limpiar y recompilar
make clean && make -j4
```

**Nota DFU:** Para entrar en modo DFU, mantener BOOT0, presionar RESET, soltar BOOT0.

---

## Debug Serial (UART)

```bash
# Abrir monitor serial (dejar abierto en terminal separada)
minicom -D /dev/tty.usbmodem102 -b 115200
minicom -D /dev/tty.usbmodem1102 -b 115200

# Buscar puertos serie
ls /dev/tty.usb*     

# Para salir de minicom: Ctrl+A luego X
```

---

## Conexiones ST-Link

```
ST-Link          WeAct Board
--------         -----------
  GND    ─────   GND
  SWCLK  ─────   PA14 (SWCLK)
  SWDIO  ─────   PA13 (SWDIO)
  3.3V   ─────   3.3V (opcional)
  RST    ─────   NRST (opcional, recomendado)
  TX     ─────   PA8  (UART7_RX) ← para debug serial
  RX     ─────   PA15 (UART7_TX) ← para debug serial
```

```bash
# Verificar conexion ST-Link
st-info --probe
```

---

## VS Code Tasks (Cmd+Shift+B)

| Task | Descripcion |
|------|-------------|
| Build | Compilar (default) |
| Clean Build | Limpiar + compilar |
| Flash (ST-Link) | Flash via SWD |
| Flash (DFU) | Flash via USB bootloader |
| Build + Flash (DFU) | Todo junto |

---

## Webserver (para DS9)

```bash
cd webserver
./run.sh

# Detener: Ctrl+C
```

---

## Archivos Importantes

| Archivo | Descripcion |
|---------|-------------|
| `firmware/Core/Src/main.c` | Init, clocks 480MHz, DMA, callbacks, main loop |
| `firmware/Core/Src/uart_protocol.c` | Parser binario ESP32→STM32 |
| `firmware/Core/Inc/uart_protocol.h` | Protocol defines, ds_config_t |
| `firmware/Core/Src/pcm1690.c` | Driver PCM1690 DAC |
| `firmware/Core/Src/epaper.c` | Driver SSD1681 e-paper |
| `firmware/STM32H743XX_FLASH.ld` | Linker script (.dma_buffer -> RAM_D2) |
| `firmware/Makefile` | Build config |
| `uart-stm32-protocol.md` | Especificacion protocolo binario |
