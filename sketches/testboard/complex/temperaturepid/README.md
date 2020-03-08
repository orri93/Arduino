# Complex Test board
## Temperature PID controller

### Types

| Type     | Size    | C type   | Description                            |
|----------|--------:|----------|----------------------------------------|
| Bits     |  8 bits | uint8_t  | Bitwise 8 bits                         |
| Real     | 32 bits | float    | General 32 bit floating point number   |
| Unsigned | 16 bits | uint16_t | General 16 bit unsigned integer number |

### Modbus

#### Coils

| Number | Address | Section | Variable | Description    |
|-------:|--------:|---------|----------|----------------|
|  00001 |  0x0000 | PID     | PonE     | The PID P on E |


#### Discrete Input

| Number | Address | Section | Variable | Description |
|-------:|--------:|---------|----------|-------------|
|  10001 |  0x0000 | Unused  | Unused   | Unused      |

#### Input Registry

| Number | Address | Size | Type     | Section     | Variable    | Description                     |
|-------:|--------:|-----:|----------|-------------|-------------|---------------------------------|
|  30001 |  0x0000 |    1 | Unsigned | Controller  | Output      | The Manual or Controller output |
|  30002 |  0x0001 |    2 | Real     | Measurement | Temperature | The Measured Temperature in °C  |
|  30002 |  0x0001 |    2 | Real     | Measurement | Temperature | The Measured Temperature in °C  |

#### Holding Registry

| Number | Address | Size | Type     | Section    | Variable | Description               |
|-------:|--------:|-----:|----------|------------|----------|---------------------------|
|  40001 |  0x0000 |    1 | Unsigned | Timing     | Interval | The control loop interval |
|  40002 |  0x0001 |    1 | Unsigned | Controller | Manual   | The Manual value          |
|  40003 |  0x0002 |    2 | Real     | PID        | Setpoint | The PID Setpoint          |
|  40005 |  0x0004 |    2 | Real     | PID        | Kp       | The PID Kp                |
|  40007 |  0x0006 |    2 | Real     | PID        | Ki       | The PID Ki                |
|  40009 |  0x0008 |    2 | Real     | PID        | Kd       | The PID Kd                |
|  40011 |  0x000A |    2 | Real     | PID        | Ti       | The PID Ti                |
|  40013 |  0x000C |    2 | Real     | PID        | Td       | The PID Td                |

### EEPROM

| Address | Size | Type     | Section    | Variable  | Description               |
|--------:|-----:|----------|------------|-----------|---------------------------|
| 0x0000  |    1 | Bits     | Modbus     | Coils     | The first 8 Modbus coils  |
| 0x0001  |    2 | Unsigned | Timing     | Interval  | The control loop interval |
| 0x0003  |    2 | Unsigned | Controller | Manual    | The Manual value          |
| 0x0005  |    4 | Real     | PID        | Setpoint  | The PID Setpoint          |
| 0x0009  |    4 | Real     | PID        | Kp        | The PID Kp                |
| 0x000D  |    4 | Real     | PID        | Ki/Ti     | The PID Ki or Ti          |
| 0x0011  |    4 | Real     | PID        | Kd/Td     | The PID Kd or Td          |
