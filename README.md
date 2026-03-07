# Temperature sensor on AVR (Arduino) operating in Slave Mode
This project is an arduino programmed on bare metal, a device capable of sending temperature data, managed by a master, which provides configuration and requests data. The project is part of a bigger one that will add my STM32 as role of controller, with the capacity to elaborate and send data with st-link to be able to be used as needed.

The system is built on an Arduino Uno R3, which handles temperature sensing through a 100kΩ NTC thermistor configured in a voltage divider circuit with a fixed 100kΩ resistor (98.1 kΩ). The analog input is processed by 10 bit ADC module.

The communication works with UART protocol. The data packet that would contains commands is defined by a byte variable. Each bit has a meaning:

- bit 0:    measurement unit (Kelvin = 0, Celsius = 1),
- bit 1:    state log time (no = 0, yes = 1),
- bit 2:    sampling mode (manual = 0, automatic = 1),
- bit 3-5:  number reppresenting time between two sampling (0s = 0, 10s = 1, 30s = 2, 90s = 3-8),
- bit 6:    save setting in sram,
- bit 7:    request data transmission.

In automatic sampling mode, the system samples data based on bits 3-5 and inserts each single data point into a stack. By default the memory stores up to 256 consecutive data, by doing so overwrite old data, when it reachs the limit. A master call lead to download the memory and to send it.

Sent data will be a couple of 32bit values, one for representing temperature and another for the measurement time. They will be sent as is usual with UART one byte at a time starting from MSB of time to LSB of temperature. Here an example of sending data:

```
60 08 00 00 09 00 00 00
0x00000860  0x00000009
2144        9
21.44°C     9s
```
It works for up to about 36 hours and 24 minutes, before the timer resets due to overflow and with a configuration of 90 seconds to maintain an auto-save state for up to about 6 hours before overwriting old saves.

**To do:**
- implement state command for the AVR to monitor the stack more other information,
- fix timing reset of sync or in general as required by the master,
- program my STM32 as master,
- change communication protocol to I2C.