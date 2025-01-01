# TLV320AIC31xx codec configuration utility

This is not tested on real hardware, not yet complete and likely has some rough edges.

## Host simulation

Host simulation uses a register map instead of i2c. This is initialized with the default values as per datasheet. Just run `make` to compile the `codec_simulator` binary from `src/main.cpp`. Any register operations will be printed to stdout:
~~~
INFO Set Page: 1
Read  Reg:031 (Binary: 0b00000100) Value: 0x04 AIC31XX_HPDRIVER
Write Reg:031 (Binary: 0b11000100) Value: 0xc4 AIC31XX_HPDRIVER
Read  Reg:040 (Binary: 0b00000010) Value: 0x02 AIC31XX_HPLGAIN
Write Reg:040 (Binary: 0b00000110) Value: 0x06 AIC31XX_HPLGAIN
~~~

## PIO 

The `platformio.ini` file can be used to confirm a successful `pio run`.
