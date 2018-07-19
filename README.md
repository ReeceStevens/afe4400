# AFE4400


![crates-io-shield](https://img.shields.io/crates/v/afe4400.svg)

The [AFE4400][afe4400] is a pulse oximetry analog front-end chip by TI. It controls LED switching, has
ambient light cancellation, and other very nice features for pulse oximetry applications.

The main communication interface for the AFE4400 is via SPI. However, there are also some
digital pins that can be configured for smoother operation; two are required and four are for
additional diagnostic value.

## Mandatory
- `adc_pdn`: The powerdown pin; should be held high (not floating!) for the duration of operation.
- `adc_rdy`: The "ADC Ready" pin, indicating a sample is ready for reading.
It is useful to use `adc_rdy` as the driver for an edge-triggered interrupt.

## Optional
- `daig_end`: Diagnostics complete
- `adc_done`: ADC conversion complete
- `led_err`: Connection issue with the LEDs
- `sensor_err`: Connection issue with the photodiodes

[afe4400]: http://www.ti.com/product/afe4400
