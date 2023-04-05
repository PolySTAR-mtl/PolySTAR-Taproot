# Robot Controller Configuration Potocol (draft)

## Configuration

Board A : UART 6

8 bits 115200 baud 1 stop bit & no parity bit

Little-Endian

## Protocol

General message layout

|   SOF  | `cfg_id`  | `data_len` | `data ` |  `checksum` |
|--------|-----------|------------|---------|-------------|
| 1 byte |  1 byte   |  1 byte    |  `data_len` bytes    | 2 bytes |
| Fixed value : `0xFC` |      |   Length of `data` |       | CRC-16 Checksum|

`cfg_id` possible values : 

| Code    |     Description      |  `data_len` |
|---------|----------------------|-------------|
| `0x01`  | SmoothPID            | 36          |
| `0x02`  | Friction Compensation| 8           |
| `0x03`  | Gravity Compensation | 4           |
| `0x04`  | Automatic Unjamming  | 40          |

## Configuration Messages

### SmoothPID

Value : `0x01`

Size : 36 bytes

Contents :

| Offset  | Size |   Type  |  Desc  |
|---------|------|---------|--------|
| 0       | 4    | float32 | kp |
| 4       | 4    | float32 | ki |
| 8       | 4    | float32 | kd |
| 12      | 4    | float32 | maxICumulative |
| 16      | 4    | float32 | maxOutput |
| 20      | 4    | float32 | QDerivativeKalman |
| 24      | 4    | float32 | RDerivativeKalman |
| 28      | 4    | float32 | QProportionalKalman |
| 32      | 4    | float32 | RProportionalKalman |

### Friction Compensation

Value : `0x02`

Size : 8 bytes

Contents :

| Offset  | Size |   Type  |  Desc  |
|---------|------|---------|--------|
| 0       | 4    | float32 | ks |
| 4       | 4    | float32 | velCutoff |

### Gravity Compensation

Value : `0x03`

Size : 4 bytes

Contents :

| Offset  | Size |   Type  |  Desc  |
|---------|------|---------|--------|
| 0       | 4    | float32 | kg |

### Automatic Unjamming

Value : `0x04`

Size : 40 bytes

Contents :

| Offset  | Size |   Type  |  Desc  |
|---------|------|---------|--------|
| 0       | 4    | float32 | targetDisplacement |
| 4       | 4    | uint32  | moveTime |
| 8       | 4    | uint32  | pauseAfterMoveTime |
| 12      | 4    | float32 | setPointTolerance |
| 16      | 4    | float32 | unjamThreshold |
| 24      | 4    | uint32  | maxWaitTime |
| 28      | 4    | uint32  | targetCycleCount |
| 32      | 4    | float32 | distanceTolerance |
| 36      | 2    | uint32  | temporalTolerance |

