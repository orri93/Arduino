# DHT11
## DHT11 Modbus

### Types

| Type     | Size    | C type   | Description                            |
|----------|--------:|----------|----------------------------------------|
| Bits     |  8 bits | uint8_t  | Bitwise 8 bits                         |
| Unsigned | 16 bits | uint16_t | General 16 bit unsigned integer number |
| Real     | 32 bits | float    | General 32 bit floating point number   |

### Modbus

#### Coils

| Number | Address | Section | Variable | Description |
|-------:|--------:|---------|----------|-------------|
|  00001 |  0x0000 | Unused  | Unused   | Unused      |


#### Discrete Input

| Number | Address | Section | Variable | Description |
|-------:|--------:|---------|----------|-------------|
|  10001 |  0x0000 | Unused  | Unused   | Unused      |

#### Input Registry

| Number | Address | Size | Type     | Section     | Variable        | Description                    |
|-------:|--------:|-----:|----------|-------------|-----------------|--------------------------------|
|  30001 |  0x0000 |    1 | Unsigned | DHT11       | Status          | The DHT11 Status value         |
|  30002 |  0x0001 |    1 | Unsigned | DHT11       | Raw Humidity    | The DHT11 Raw humidity value   |
|  30003 |  0x0002 |    1 | Unsigned | DHT11       | Raw Temperature | The DHT11 Temperature          |
|  30004 |  0x0003 |    2 | Real     | DHT11       | Humidity        | The Humidity                   |
|  30006 |  0x0005 |    2 | Real     | DHT11       | Temperature     | The Measured Temperature in °C |

#### Holding Registry

| Number | Address | Size | Type     | Section    | Variable | Description  |
|-------:|--------:|-----:|----------|------------|----------|--------------|
|  40001 |  0x0000 |    0 |  Unused  |   Unused   | Unused   | Unused       |


### Display

| Line | Variable    | Description               |
|-----:|-------------|---------------------------|
|    1 | Temperature | The measured temperature  |
|    2 | Humidity    | The Humidity              |

### DHT11

#### Status

| Value | Name     | Description  |
|------:|----------|--------------|
|     0 | Success  | Success      |
|     1 | Timeout  | Timeout      |
|     2 | Checksum | Checksum     |

### Build

`Build-Arduino -Board uno -CompilerCppExtraFlags -DMODBUS_HANDLER_INTERFACE`