# Complex Test board
## Temperature PID controller

### Types

| Type     | Size    | C type   | Description                            |
|----------|--------:|----------|----------------------------------------|
| Bits     |  8 bits | uint8_t  | Bitwise 8 bits                         |
| Unsigned | 16 bits | uint16_t | General 16 bit unsigned integer number |
| Real     | 32 bits | float    | General 32 bit floating point number   |

### Modbus

#### Coils

| Number | Address | Section | Variable | Description    |
|-------:|--------:|---------|----------|----------------|
|  00001 |  0x0000 | PID     | PonE     | The PID P on E |
|  00002 |  0x0001 | PID     | Tune     | Tune with time |

#### Discrete Input

| Number | Address | Section | Variable | Description |
|-------:|--------:|---------|----------|-------------|
|  10001 |  0x0000 | Unused  | Unused   | Unused      |

#### Input Registry

| Number | Address | Size | Type     | Section     | Variable    | Description                     |
|-------:|--------:|-----:|----------|-------------|-------------|---------------------------------|
|  30001 |  0x0000 |    1 | Unsigned | Controller  | Output      | The Manual or Controller output |
|  30002 |  0x0001 |    2 | Real     | Measurement | Temperature | The Measured Temperature in °C  |
|  30004 |  0x0003 |    2 | Real     | PID         | Ti          | The Ki * time from the tuning   |
|  30006 |  0x0005 |    2 | Real     | PID         | Td          | The Kd / time from the tuning   |
|  30008 |  0x0007 |    2 | Real     | PID         | Output sum  | The internal output sum         |

#### Holding Registry

| Number | Address | Size | Type     | Section    | Variable | Description               |
|-------:|--------:|-----:|----------|------------|----------|---------------------------|
|  40001 |  0x0000 |    1 | Unsigned | Timing     | Interval | The control loop interval |
|  40002 |  0x0001 |    1 | Unsigned | Controller | Manual   | The Manual value          |
|  40003 |  0x0002 |    2 | Real     | PID        | Setpoint | The PID Setpoint          |
|  40005 |  0x0004 |    2 | Real     | PID        | Kp       | The PID Kp                |
|  40007 |  0x0006 |    2 | Real     | PID        | Ki/Ti    | The PID Ki/Ti             |
|  40009 |  0x0008 |    2 | Real     | PID        | Kd/Td    | The PID Kd/Td             |
|  40011 |  0x000A |    2 | Double   | Range      | MinSens  | The min range for the sens|
|  40013 |  0x000C |    2 | Double   | Range      | MaxSens  | The max range for the sens|
|  40015 |  0x000E |    1 | Unsigned | PID tune   | TimeTune | Time tune                 |
|  40016 |  0x000F |    1 | Unsigned | Force      | Force    | Force mode                |

### EEPROM

| Address | Size | Type     | Section    | Variable   | Description                      |
|--------:|-----:|----------|------------|------------|----------------------------------|
| 0x0000  |    1 | Bits     | Modbus     | Coils      | The first 8 Modbus coils         |
| 0x0001  |    2 | Unsigned | Timing     | Interval   | The control loop interval        |
| 0x0003  |    2 | Unsigned | Controller | Manual     | The Manual value                 |
| 0x0005  |    4 | Real     | PID        | Setpoint   | The PID Setpoint                 |
| 0x0009  |    4 | Real     | PID        | Kp         | The PID Kp                       |
| 0x000D  |    4 | Real     | PID        | Ki/Ti      | The PID Ki or Ti                 |
| 0x0011  |    4 | Real     | PID        | Kd/Td      | The PID Kd or Td                 |
| 0x0013  |    4 | Real     | Range      | Min sensor | The minimum range for the sensor |
| 0x0015  |    4 | Real     | Range      | Max sensor | The maximum range for the sensor |

### Display

| Line | Variable    | Description               |
|-----:|-------------|---------------------------|
|    1 | Temperature | The measured temperature  |
|    2 |             | The last command line     |

### PID

#### Time tune

LowByte is the PID tune time unit
HighByte is the PID time tune variable time unit

| Value | Name         | Description               |
|------:|--------------|---------------------------|
|     0 | Default      | Use factory setting       |
|     1 | Milliseconds | Time tuning are in ms     |
|     2 | Seconds      | Time tuning are in s      |
|     1 | Minutes      | Time tuning are in min    |

### Build

`Build-Arduino -Board uno -CompilerCppExtraFlags -DMODBUS_HANDLER_INTERFACE`