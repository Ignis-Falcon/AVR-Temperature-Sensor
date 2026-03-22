# AVR-STM32 Temperature Sensor System — Master/Slave UART
This project is an arduino programmed on bare metal, a device capable of sending temperature data, managed by a master, which provides configuration and requests data. The STM32 as role of controller, collecting temperature samples from the slave and outputting data via serial.

## Slave
The system is built on an Arduino Uno R3, which handles temperature sensing through a 100kΩ NTC thermistor configured in a voltage divider circuit with a fixed 100kΩ resistor (98.1 kΩ). The analog input is processed by 10 bit ADC module.

The communication works with UART protocol. The data packet that would contains commands is defined by a byte variable. Each bit has a meaning:

- bit 0:    measurement unit (Kelvin = 0, Celsius = 1),
- bit 1:    state log time (no = 0, yes = 1),
- bit 2:    sampling mode (manual = 0, automatic = 1),
- bit 3-5:  number representing time between two sampling (0s = 0, 10s = 1, 30s = 2, 90s = 3-8),
- bit 6:    save setting in sram,
- bit 7:    request data transmission.

In automatic sampling mode, the system samples data based on bits 3-5 and inserts each single data point into a stack. By default the memory stores up to 256 consecutive data, by doing so overwrite old data, when it reachs the limit. A master call lead to download the memory and to send it.

**NOTE:** Always in automatic mode setted, will go reset stack and timing log, because in synching all temperature data are converted by an unic measurament unit.

Sent data will be a couple of 32bit values, one for representing temperature and another for the measurement time. They will be sent as is usual with UART one byte at a time starting from MSB of time to LSB of temperature. Here an example of sending data:

```
60 08 00 00 09 00 00 00
0x00000860  0x00000009
2144        9
21.44°C     9s
```
It works for up to about 36 hours and 24 minutes, before the timer resets due to overflow and with a configuration of 90 seconds to maintain an auto-save state for up to about 6 hours before overwriting old saves.

To query status of the stack the master can send 0x00 value: It receives a status of the stack, indicating the current number of elements occupied and remaining time before overwriting.

## Master
The master is programmed using STM32 HAL and manages communication with the slave via UART1. It operates in two modes depending on the sampling configuration:
- **Slave mode**: the master periodically queries the slave stack status and synchronizes data when the stack reaches a configured percentage.
- **Master mode**: the master requests a single sample from the slave at a fixed interval defined by `master_sampling_time`.

Received data is stored in an internal ring buffer of 512 elements. When the buffer is full, the data is output via UART2 and can be readed by any serial terminal.

**To do:**
- [x] ~~implement state command for the AVR to monitor the stack more other information,~~
- [x] ~~fix timing reset of sync or in general as required by the master,~~
- [x] ~~program my STM32 as master,~~
- [ ] RTC on STM32 for absolute timestamp and sync time via UART2 by pc,
- [ ] CubeMonitor:
    - set configuration,
    - view history,
    - save offline data,
- [ ] manual mode, controlled by user,
- [ ] optimization replacing HAL with LL one module at a time,
- [ ] change communication protocol to I2C.


## Licensing

This project is released under the **MIT License** (see the `LICENSE` file in the root directory).

This repository includes third-party libraries required for STM32 hardware functionality:

- **STM32 HAL Drivers:** Developed by STMicroelectronics. Distributed under the terms of the **BSD-3-Clause License** (documentation available within the `master/Drivers/STM32G4xx_HAL_Driver` directory).
- **CMSIS (Cortex Microcontroller Software Interface Standard):** Developed by ARM. Distributed under the terms of the **Apache 2.0 License** (documentation available within the `master/Drivers/CMSIS/` directory).

All rights to third-party files remain with their respective owners.