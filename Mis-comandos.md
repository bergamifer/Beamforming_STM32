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
| `firmware/Core/Src/main.c` | Codigo principal |
| `firmware/Weact_test_audio.ioc` | Config CubeMX |
| `firmware/STM32H743XX_FLASH.ld` | Linker script (incluye .dma_buffer) |
| `firmware/Makefile` | Build config |
