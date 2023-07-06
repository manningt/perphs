# Perphs
Provides an API from the Raspberry Pi to the Soc
* Currently uses the UART, but could use SPI
* write to the DACs, GPIO and PWMs (motor control)
* get sent 2 ADC channel readings, 2 pulse counter readings, ball switch state (needs to be implemented to get the latched state), fault state information (an enum?) and maybe other state info.

# Details
This code uses ESP-IDF (Integrated Development Framework) which uses Free-RTOS. Currently there is only 1 active task 'UART' which reads command data from the UART and either sends back the current values of the ADCs, pulse counters and carousel state, or updates the motor values (PWM and DAC).

This code could be used as a model for other types of SoC.

Currently the implementation uses combines 6 bits of 2 bytes read from the UART to support 12 bit values for the motor controls (DAC, ADC, PWM and Tach).  This was done to minimize the number of bytes transferred in a message.  The upper 2 bits are used to indicate whether the byte contains data or a command.  Checking has to be added to do data/command bit verification and deal with reading the UART when the RPi and SoC are not in-sync.

It works, but there may be a better implementation for message encoding/decoding.