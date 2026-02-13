# Protocolo UART: ESP32-S2 Mini -> STM32H743

## Arquitectura General

```
[Servidor Flask] <--WiFi--> [ESP32-S2 Mini] <--UART--> [STM32H743]
   (puerto 5001)              (este doc)               (tu parte)
```

El ESP32-S2 Mini recibe comandos JSON desde un servidor web Flask via HTTP POST,
los parsea y los envia al STM32H743 por UART usando un protocolo binario compacto.

## Conexion Fisica

| Señal | ESP32-S2 Mini | STM32H743 | Notas |
|-------|---------------|-----------|-------|
| TX    | GPIO 1        | RX (pin UART del STM32) | ESP32 transmite |
| RX    | GPIO 2        | TX (pin UART del STM32) | ESP32 recibe |
| GND   | GND           | GND       | Obligatorio |

**IMPORTANTE**: No conectar VCC entre las placas si ya tienen alimentacion independiente.
Ambas placas trabajan a 3.3V asi que los niveles logicos son compatibles directamente.

## Configuracion UART

| Parametro | Valor |
|-----------|-------|
| Baudrate  | 115200 |
| Data bits | 8 |
| Parity    | None |
| Stop bits | 1 |
| Flow ctrl | None |

## Protocolo Binario

### Trama de datos (ESP32 -> STM32)

```
| Offset | Campo       | Tamaño  | Valor/Descripcion                    |
|--------|-------------|---------|--------------------------------------|
| 0      | Header      | 2 bytes | 0xAA 0x55 (fijo)                    |
| 2      | Command     | 1 byte  | 0x01 = update_config                |
| 3      | Length      | 2 bytes | Largo del payload (little endian)    |
| 5      | Payload     | N bytes | Datos (ver detalle abajo)            |
| 5+N    | Checksum    | 1 byte  | XOR de todos los bytes del payload   |
| 6+N    | Footer      | 2 bytes | 0x0D 0x0A (fijo)                    |
```

### Detalle del Payload para CMD 0x01 (update_config)

Largo total del payload: **52 bytes**

```
| Offset | Campo           | Tamaño   | Tipo     | Descripcion                        |
|--------|-----------------|----------|----------|------------------------------------|
| 0      | master_gain     | 1 byte   | uint8    | Ganancia master (0-255 = 0.0-1.0)  |
| 1      | mute_global     | 1 byte   | uint8    | 0=unmuted, 1=muted                 |
| 2      | delays[0..15]   | 32 bytes | uint16[] | delay_samples por canal (LE)       |
| 34     | gains[0..15]    | 16 bytes | uint8[]  | gain por canal (0-255 = 0.0-1.0)   |
| 50     | mutes           | 2 bytes  | uint16   | Bitfield de mutes (LE)             |
```

**LE = Little Endian** (byte menos significativo primero)

### Ejemplo de trama completa

Para una configuracion con:
- master_gain = 0.8 (= 204 = 0xCC)
- mute_global = 0
- Canal 0: delay=1440 (0x05A0), gain=1.0 (0xFF), mute=0
- Canal 1: delay=1560 (0x0618), gain=0.95 (0xF2), mute=0
- Canales 2-15: todo ceros

```
Trama hex:
AA 55 01 34 00 CC 00 A0 05 18 06 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
FF F2 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 [XOR] 0D 0A
```

Donde:
- `AA 55` = Header
- `01` = Command (update_config)
- `34 00` = Length (52 = 0x0034, little endian)
- `CC` = master_gain (204)
- `00` = mute_global (no)
- `A0 05` = delays[0] = 1440 (little endian)
- `18 06` = delays[1] = 1560 (little endian)
- ... (canales 2-15 en cero)
- `FF` = gains[0] = 255
- `F2` = gains[1] = 242
- ... (canales 2-15 en cero)
- `00 00` = mutes bitfield (ningun canal muteado)
- `[XOR]` = checksum XOR de los 52 bytes del payload
- `0D 0A` = Footer

## Respuesta ACK/NACK (STM32 -> ESP32)

El STM32 debe responder con una trama simple despues de recibir un comando:

### ACK (comando recibido y aplicado OK)

```
| Offset | Campo   | Tamaño  | Valor    |
|--------|---------|---------|----------|
| 0      | Header  | 2 bytes | 0xAA 0x55|
| 2      | Command | 1 byte  | 0x06 (ACK)|
| 3      | Footer  | 2 bytes | 0x0D 0x0A|
```

Trama: `AA 55 06 0D 0A` (5 bytes)

### NACK (error)

```
| Offset | Campo   | Tamaño  | Valor     |
|--------|---------|---------|-----------|
| 0      | Header  | 2 bytes | 0xAA 0x55 |
| 2      | Command | 1 byte  | 0x15 (NACK)|
| 3      | Footer  | 2 bytes | 0x0D 0x0A |
```

Trama: `AA 55 15 0D 0A` (5 bytes)

**Timeout**: El ESP32 espera 500ms por una respuesta. Si no llega, asume que no hay STM32 conectado
y reporta "warning" al servidor web (pero no falla).

## Algoritmo de recepcion sugerido para STM32

```c
// Pseudocodigo para STM32 HAL

#define HEADER_1 0xAA
#define HEADER_2 0x55
#define CMD_UPDATE_CFG 0x01
#define CMD_ACK  0x06
#define CMD_NACK 0x15
#define FOOTER_1 0x0D
#define FOOTER_2 0x0A

typedef struct {
    uint8_t  master_gain;
    uint8_t  mute_global;
    uint16_t delays[16];
    uint8_t  gains[16];
    uint16_t mutes;  // bitfield
} ds_config_t;

// Estado de la maquina de estados para parseo
typedef enum {
    WAIT_HEADER_1,
    WAIT_HEADER_2,
    WAIT_COMMAND,
    WAIT_LENGTH_L,
    WAIT_LENGTH_H,
    WAIT_PAYLOAD,
    WAIT_CHECKSUM,
    WAIT_FOOTER_1,
    WAIT_FOOTER_2
} parse_state_t;

// Procesar byte por byte (llamar desde UART RX interrupt o DMA callback)
void process_byte(uint8_t byte) {
    static parse_state_t state = WAIT_HEADER_1;
    static uint8_t command;
    static uint16_t payload_len;
    static uint8_t payload[64];
    static uint16_t payload_idx;
    static uint8_t checksum_received;

    switch (state) {
    case WAIT_HEADER_1:
        if (byte == HEADER_1) state = WAIT_HEADER_2;
        break;

    case WAIT_HEADER_2:
        state = (byte == HEADER_2) ? WAIT_COMMAND : WAIT_HEADER_1;
        break;

    case WAIT_COMMAND:
        command = byte;
        state = WAIT_LENGTH_L;
        break;

    case WAIT_LENGTH_L:
        payload_len = byte;
        state = WAIT_LENGTH_H;
        break;

    case WAIT_LENGTH_H:
        payload_len |= (byte << 8);
        payload_idx = 0;
        state = (payload_len > 0 && payload_len <= sizeof(payload))
                ? WAIT_PAYLOAD : WAIT_HEADER_1;
        break;

    case WAIT_PAYLOAD:
        payload[payload_idx++] = byte;
        if (payload_idx >= payload_len) state = WAIT_CHECKSUM;
        break;

    case WAIT_CHECKSUM:
        checksum_received = byte;
        state = WAIT_FOOTER_1;
        break;

    case WAIT_FOOTER_1:
        state = (byte == FOOTER_1) ? WAIT_FOOTER_2 : WAIT_HEADER_1;
        break;

    case WAIT_FOOTER_2:
        state = WAIT_HEADER_1;
        if (byte != FOOTER_2) break;

        // Verificar checksum
        uint8_t calc_xor = 0;
        for (int i = 0; i < payload_len; i++) calc_xor ^= payload[i];

        if (calc_xor == checksum_received && command == CMD_UPDATE_CFG) {
            // Parsear payload a ds_config_t
            ds_config_t *cfg = (ds_config_t *)payload;
            // NOTA: los uint16 ya vienen en little endian, STM32 es LE tambien
            // asi que se puede castear directamente si el struct esta packed

            apply_config(cfg);  // Tu funcion para aplicar delays/gains
            send_ack();         // Enviar 0xAA 0x55 0x06 0x0D 0x0A
        } else {
            send_nack();        // Enviar 0xAA 0x55 0x15 0x0D 0x0A
        }
        break;
    }
}

// Enviar ACK
void send_ack(void) {
    uint8_t ack[] = {0xAA, 0x55, 0x06, 0x0D, 0x0A};
    HAL_UART_Transmit(&huartX, ack, 5, 100);
}

// Enviar NACK
void send_nack(void) {
    uint8_t nack[] = {0xAA, 0x55, 0x15, 0x0D, 0x0A};
    HAL_UART_Transmit(&huartX, nack, 5, 100);
}
```

## Datos originales del JSON (referencia)

El ESP32 recibe esto del servidor Flask y lo convierte al protocolo binario:

```json
{
  "command": "update_config",
  "device_id": "DS_01",
  "mute": false,
  "master_gain": 0.8,
  "focal_distance": 3.5,
  "channels": [
    {
      "channel_num": 0,
      "delay_samples": 1440,
      "channel_gain": 1.0,
      "mute": 0
    },
    {
      "channel_num": 1,
      "delay_samples": 1560,
      "channel_gain": 0.95,
      "mute": 0
    }
  ]
}
```

### Conversion de valores

| Campo JSON       | Tipo JSON | Conversion a binario                       |
|------------------|-----------|-------------------------------------------|
| master_gain      | float 0-1 | uint8: valor * 255                         |
| mute (global)    | bool      | uint8: 0 o 1                               |
| delay_samples    | int       | uint16 little endian (sin conversion)       |
| channel_gain     | float 0-1 | uint8: valor * 255                         |
| mute (canal)     | int 0/1   | bit N del bitfield uint16 mutes             |
| focal_distance   | float     | No se envia por UART (solo lo usa el server)|

## Resumen de comandos

| ID   | Nombre        | Direccion       | Payload |
|------|---------------|-----------------|---------|
| 0x01 | update_config | ESP32 -> STM32  | 52 bytes (config completa) |
| 0x06 | ACK           | STM32 -> ESP32  | Sin payload |
| 0x15 | NACK          | STM32 -> ESP32  | Sin payload |

## Notas importantes

1. **Nivel logico**: Ambas placas son 3.3V, conexion directa sin level shifter
2. **GND comun**: Es obligatorio conectar GND entre ambas placas
3. **Sin flow control**: No se usa RTS/CTS
4. **Byte order**: Todo little endian (nativo de ARM Cortex-M y Xtensa)
5. **El struct ds_config_t se puede castear directamente** desde el payload si se declara como `__attribute__((packed))` ya que ambos procesadores son little endian
6. **focal_distance no se envia por UART** - solo lo usa el servidor web para calcular los delays
