# TLV320AIC31xx codec configuration utility

This library allows configuration of the Texas Instruments TLV320AIC31xx codecs on Arduino platforms. It has been tested with the TLV320AIC3100 on a ACEV-1B eval board (i2c test points and ground hooked up to an esp32 dev kit board). It does not support all codec functions yet (like mic AGC) and likely has some rough edges.

![screenshot](images/eval.jpg)

## Host simulation

Host simulation uses a register map instead of i2c. This is initialized with the default values as per datasheet. Just run `make` to compile the `codec_simulator` binary from `examples/main.cpp`. Any register operations will be printed to stdout:
~~~
INFO Set Page: 1
Read  Reg:031 (Binary: 0b00000100) Value: 0x04 AIC31XX_HPDRIVER
Write Reg:031 (Binary: 0b11000100) Value: 0xc4 AIC31XX_HPDRIVER
Read  Reg:040 (Binary: 0b00000010) Value: 0x02 AIC31XX_HPLGAIN
Write Reg:040 (Binary: 0b00000110) Value: 0x06 AIC31XX_HPLGAIN
~~~

## PIO

The `platformio.ini` should be self-explantory. I2C pins are defined as build flags.

## Example

The setup method in `examples/main.cpp` shows all required calls to enable DAC, headphone, speakers, clocking via PLL from BLCK for 44,1kHz 16bit.

## Integration in platform.io project

Usage of this library is straightforward. Add the library to the `lib_deps`:
~~~
lib_deps=
	https://github.com/haklein/tlv320aic31xx.git
~~~
Include the header file and instantiate the class:
~~~
#include "tlv320aic31xx_codec.h"

TLV320AIC31xx codec(&Wire);
~~~

Call the `begin()` method to reset the codec after setting up TwoWire and then configure the codec as per requirements:
~~~
    Wire.begin(SDA_PIN, SCL_PIN);
    codec.begin(); // reset codec
    sleep(1);
    codec.setWordLength(AIC31XX_WORD_LEN_16BITS);
    codec.setCLKMUX(AIC31XX_PLL_CLKIN_BCLK, AIC31XX_CODEC_CLKIN_PLL);
    codec.setPLL(1, 2, 32, 0); // uint8_t pll_p, uint8_t pll_r, uint8_t pll_j, uint16_t pll_d
    codec.setNDACVal(8);
    codec.setNDACPower(true);
    codec.setMDACVal(2);
    codec.setMDACPower(true);
    [...]
~~~

## Usage notes

### Headphone detection

When generating the MCLK from BCLK (as per example above) the clock for the timers need to be generated internally. This is required for debouncing and the headset detection doesn't work without the timers. There is no high level method for this yet but it can be set for the time being via:
~~~
    codec.modifyRegister(AIC31XX_TIMERDIVIDER, AIC31XX_TIMER_SELECT_MASK, 0);
~~~    

Detection can be configured to trigger the GPIO output and enabled via:
~~~
      codec.enableHeadsetDetect();
      codec.setHSDetectInt1(true);
~~~

This can then be handled via ISR (assuming `CONFIG_TLV320AIC3100_INT` would be defined to the connected GPIO pin):
~~~
 pinMode(CONFIG_TLV320AIC3100_INT, INPUT);
 attachInterrupt(CONFIG_TLV320AIC3100_INT, codec_isr, RISING);
~~~

Please be aware that the codec won't send any further interrupts until the interrupt flag register has been read:
~~~
  if (codec.readRegister(AIC31XX_INTRDACFLAG) & AIC31XX_HSPLUG) { // bit 4 is set on headset related interrupts
    Serial.println("AIC31XX: Headset plug interrupt triggered");
  }
~~~

[![PlatformIO Registry](https://badges.registry.platformio.org/packages/haklein/library/tlv320aic31xx.svg)](https://registry.platformio.org/libraries/haklein/tlv320aic31xx)
