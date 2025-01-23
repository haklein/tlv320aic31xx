// Arduino-compatible I2C codec configuration class implementation for TLV320AIC31xx codec
//
#include <iostream>
#include <map>
#include <iomanip> // For hex formatting
#include <bitset>
#include <math.h>

#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#define LOG(x) Serial.print(x)
#define LOG_LN(x) Serial.println(x)
#else
#include <stdint.h>
#define LOG(x) std::cout << x
#define LOG_LN(x) std::cout << x << std::endl
#endif

#include "tlv320aic31xx_codec.h"
#include "tlv320aic31xx_regs.h"

// Extract page and register address from AIC31XX_REG macro
uint8_t TLV320AIC31xx::getPage(uint16_t reg) {
    return reg / 128;
}

uint8_t TLV320AIC31xx::getRegister(uint16_t reg) {
    return reg % 128;
}
const char* lookupRegisterName(uint16_t address) {
    for (const auto& entry : registerTable) {
        if (entry.reg == address) {
            return entry.name;
        }
    }
    return "Unknown Register";
}

#ifndef ARDUINO
#include <map>
#include <vector>
#include <iostream>
#include <iomanip>

// Simulated register structure for multiple pages
std::map<uint8_t, std::vector<uint8_t>> simulatedRegisters;

uint8_t getPage(uint16_t reg) {
    return reg / 128;
}

uint8_t getRegister(uint16_t reg) {
    return reg % 128;
}


// Write to the simulated registers
void writeSimulatedRegister(uint16_t reg, uint8_t value) {
    uint8_t page = getPage(reg);
    uint8_t regAddr = getRegister(reg);

    if (simulatedRegisters.find(page) != simulatedRegisters.end() && regAddr < 128) {
        simulatedRegisters[page][regAddr] = value;
        /* std::cout << "Simulated Write: Page " << (int)page
                  << ", Register 0x" << std::hex << (int)regAddr
                  << ", " << lookupRegisterName(reg)
                  << ", Value 0x" << std::hex << (int)value << "\n"; */
    } else {
        std::cerr << "Simulated Write Error: Invalid page or register address\n";
    }
}

// Read from the simulated registers
uint8_t readSimulatedRegister(uint16_t reg) {
    uint8_t page = getPage(reg);
    uint8_t regAddr = getRegister(reg);

    if (simulatedRegisters.find(page) != simulatedRegisters.end() && regAddr < 128) {
        uint8_t value = simulatedRegisters[page][regAddr];
   /*     std::cout << "Simulated Read: Page " << (int)page
                  << ", Register 0x" << std::hex << (int)regAddr
                  << ", Value 0x" << std::hex << (int)value << "\n";*/
        return value;
    } else {
        std::cerr << "Simulated Read Error: Invalid page or register address\n";
        return 0; // Return a default value on error
    }
}

// Helper to initialize the simulated registers
void initializeSimulatedRegisters() {
    for (uint8_t page = 0; page < 3; ++page) { // Assuming up to 3 pages for simulation
        simulatedRegisters[page] = std::vector<uint8_t>(128, 0); // Each page has 128 registers
    }
    for (uint16_t i = 0; i<sizeof(aic31xx_reg_defaults)/sizeof(aic31xx_reg_defaults[0]);i++){
	    std::cout << "Init val: " << (int)getPage(aic31xx_reg_defaults[i].reg) << ":" << (int)getRegister(aic31xx_reg_defaults[i].reg)  << " " << aic31xx_reg_defaults[i].val << std::endl;
	    writeSimulatedRegister(aic31xx_reg_defaults[i].reg, aic31xx_reg_defaults[i].val);
    }

}

#endif

// Function to set the active page
void TLV320AIC31xx::setPage(uint8_t page) {
    if (this->current_page != page) {
#ifdef ARDUINO
		this->twowire->beginTransmission(TLV320AIC31XX_I2C_ADDRESS);
		this->twowire->write(PAGE_CTRL_REGISTER);
		this->twowire->write(page);
		this->twowire->endTransmission();
#endif
		LOG("INFO Set Page: ");
		LOG_LN((int)page);
		current_page = page;
	}
}

uint8_t TLV320AIC31xx::convertDACVolumeToRegisterValue(float dB) {
    // Clamp dB to valid range
    if (dB > 24.0f) dB = 24.0f; // Max volume: +24 dB
    if (dB < -63.5f) dB = -63.5f; // Min volume: -63.5 dB

    // Convert dB to register value (0.5 dB steps)
    int16_t value = static_cast<int16_t>(dB * 2); // Multiply by 2 to handle 0.5 dB steps

    // Adjust for the unsigned register format:
    // - Positive dB values are mapped directly (0x00 to 0x30 for 0 dB to +24 dB).
    // - Negative dB values are mapped to 2's complement (0xFF to 0x82 for -0.5 dB to -63 dB).
    uint8_t regValue = (value >= 0) ? value : (0x100 + value);

    // Ensure that reserved values (0x80) are not used
    if (regValue == 0x80) regValue = 0x81; // Adjust to closest valid value (-63.5 dB)

    return regValue;
}

uint8_t TLV320AIC31xx::convertADCVolumeToRegisterValue(float dB) {
    // Clamp dB to valid range
    if (dB > 20.0f) dB = 20.0f;   // Max volume: +20 dB
    if (dB < -12.0f) dB = -12.0f; // Min volume: -12 dB

    uint8_t regValue;

    if (dB < 0.0f) {
        // Negative dB: map -12.0 dB to 0x68, -0.5 dB to 0x7F
        regValue = 0x68 + static_cast<uint8_t>((dB + 12.0f) * 2);
    } else {
        // Non-negative dB: map 0.0 dB to 0x00, +20.0 dB to 0x28
        regValue = static_cast<uint8_t>(dB * 2);
    }

    return regValue;
}

uint8_t TLV320AIC31xx::convertAnalogGainToRegisterValue(float gainDb) {
    // Clamp gain to valid range
    if (gainDb > analogGainTable[0]) {
	LOG_LN("clamp to 0dB");
	LOG_LN(analogGainTable[0]);
	std::cout << "Gaintable: " << analogGainTable[0] << std::endl;
        gainDb = analogGainTable[0]; // Maximum gain: 0.0 dB
    }
    if (gainDb < analogGainTable[127]) {
	LOG_LN("clamp to -78.3dB");
	LOG_LN(analogGainTable[127]);
	std::cout << "Gaintable: " << analogGainTable[127] << std::endl;
	std::cout << "Gaintable: " << analogGainTable[20] << std::endl;
	std::cout << "Gaintable: " << analogGainTable[40] << std::endl;
	gainDb = analogGainTable[127]; // Minimum gain: -78.3 dB
	LOG_LN(analogGainTable[127]);
    }

    // Find the closest match in the table
    uint8_t closestIndex = 0;
    float smallestDifference = fabs(gainDb - analogGainTable[0]);

    for (uint8_t i = 1; i < sizeof(analogGainTable) / sizeof(analogGainTable[0]); ++i) {
        float difference = fabs(gainDb - analogGainTable[i]);
        if (difference < smallestDifference) {
            closestIndex = i;
            smallestDifference = difference;
        }
    }

    return closestIndex;
}

uint8_t TLV320AIC31xx::convertPgaGainToRegisterValue(float gainDb) {
    // Clamp gain to the valid range [0, 9]
    if (gainDb < 0.0f) {
        gainDb = 0.0f;
    } else if (gainDb > 9.0f) {
        gainDb = 9.0f;
    }

    // Convert directly to register value
    return static_cast<uint8_t>(gainDb);
}

uint8_t TLV320AIC31xx::convertMicPgaGainToRegisterValue(float gainDb) {
    // Clamp gain to the valid range [0.0, 59.5]
    if (gainDb < 0.0f) {
        gainDb = 0.0f;
    } else if (gainDb > 59.5f) {
        gainDb = 59.5f;
    }

    // Convert directly to register value (0.5 dB steps)
    return static_cast<uint8_t>(gainDb * 2);
}


// Constructor
#ifdef ARDUINO
TLV320AIC31xx::TLV320AIC31xx(TwoWire *twowire) {
	this->twowire = twowire;
#else
TLV320AIC31xx::TLV320AIC31xx() {
	initializeSimulatedRegisters();
#endif
}

// Initialize the codec
bool TLV320AIC31xx::begin() {
  LOG_LN("initialize codec");
  if (!isConnected()) return false;
  reset();
#ifdef ARDUINO
  delay(10); // 10ms startup delay to stabilize PLL
#endif
  modifyRegister(AIC31XX_IFACE1, AIC31XX_IFACE1_DATATYPE_MASK, AIC31XX_I2S_MODE); // i2s (default)
  return true;
}

bool TLV320AIC31xx::isConnected() {
#ifdef ARDUINO
  this->twowire->beginTransmission(TLV320AIC31XX_I2C_ADDRESS);
  if (this->twowire->endTransmission() != 0)
    return false; // codec did not ACK
#endif
  return true;
}

// Function to write a single register
bool TLV320AIC31xx::writeRegister(uint16_t reg, uint8_t value) {
    uint8_t page = getPage(reg);
    uint8_t regAddr = getRegister(reg);

    setPage(page);

#ifdef ARDUINO
    Serial.print("Write Reg: ");
    Serial.print(regAddr, DEC);
    Serial.print(" (Binary: 0b");
    Serial.print(value, BIN);
    Serial.print(") Value: 0x");
    Serial.print(value, HEX);
    Serial.print(" ");
    Serial.println(lookupRegisterName(reg));
    this->twowire->beginTransmission(TLV320AIC31XX_I2C_ADDRESS);
    this->twowire->write(regAddr);
    this->twowire->write(value);
    if (this->twowire->endTransmission() != 0) {
      Serial.println("ERROR: cannot write i2c register");
      return false;
    }
#else
    // Simulate register write
    writeSimulatedRegister(reg, value);
    LOG("Write Reg:");
    LOG(std::dec << std::setw(3) << std::setfill('0') << (int)regAddr);
    LOG(" (Binary: 0b");
    LOG(std::bitset<8>(value));
    LOG(") Value: 0x");
    LOG(std::hex << std::setw(2) << std::setfill('0') << (int)value);
    LOG_LN(" " << lookupRegisterName(reg));
#endif
    return true;
}

// Function to read a single register
uint8_t TLV320AIC31xx::readRegister(uint16_t reg) {
    uint8_t page = getPage(reg);
    uint8_t value = 0;
    setPage(page);
    uint8_t regAddr = getRegister(reg);

#ifdef ARDUINO
    this->twowire->beginTransmission(TLV320AIC31XX_I2C_ADDRESS);
    this->twowire->write(regAddr);
    this->twowire->endTransmission(false); // Send repeated start

    this->twowire->requestFrom((uint8_t)TLV320AIC31XX_I2C_ADDRESS, (size_t)1);
    if (this->twowire->available()) {
        value = this->twowire->read();
	Serial.print("Read Reg:  ");
	Serial.print(regAddr, DEC);
	Serial.print(" (Binary: 0b");
	Serial.print(value, BIN);
	Serial.print(") Value: 0x");
	Serial.print(value, HEX);
	Serial.print(" ");
	Serial.println(lookupRegisterName(reg));
        return value;
    } else {
	Serial.println("ERROR: Cannot read register from codec");
    }
    return 0; // Return 0 if no data is available
#else
    value = readSimulatedRegister(reg);
    LOG("Read  Reg:");
    LOG(std::dec << std::setw(3) << std::setfill('0') << (int)regAddr);
    LOG(" (Binary: 0b");
    LOG(std::bitset<8>(value));
    LOG(") Value: 0x");
    LOG(std::hex << std::setw(2) << std::setfill('0') << (int)value);
    LOG_LN(" " << lookupRegisterName(reg));
    return value;
#endif
}

// Function to read, modify, and write back specific bits in a register
void TLV320AIC31xx::modifyRegister(uint16_t reg, uint8_t mask, uint8_t value) {
    uint8_t currentValue = readRegister(reg);
    uint8_t shift = 0;

    // determine left shift bit count from mask
    while (((mask >> shift) & 1) == 0 && shift < 8) {
        shift++;
    }

    // Shift the value into position
    uint8_t shiftedValue = (value << shift) & mask;
    uint8_t newValue = (currentValue & ~mask) | shiftedValue;
#ifdef ARDUINO
    Serial.print("Modifying Register: ");
    Serial.print(getRegister(reg), DEC);
    Serial.print(" [Before: 0b");
    Serial.print(currentValue, BIN);
    Serial.print("] with Mask: 0b");
    Serial.print(mask, BIN);
    Serial.print(" and Value: 0b");
    Serial.println(value, BIN);
#else
    /*
    LOG("Modifying Register ");
    LOG(std::setw(3) << std::setfill('0') << (int)getRegister(reg));
    LOG(" [Before: 0x");
    LOG(std::hex << std::setw(2) << std::setfill('0') << (int)currentValue);
    LOG(" (Binary: 0b");
    LOG(std::bitset<8>(currentValue));
    LOG(")] with Mask: 0x");
    LOG(std::hex << (int)mask);
    LOG(" (Binary: 0b");
    LOG(std::bitset<8>(mask));
    LOG(") and Value: 0x");
    LOG(std::hex << (int)value);
    LOG(" (Binary: 0b");
    LOG_LN(std::bitset<8>(value));
*/
#endif

    writeRegister(reg, newValue);
}

void TLV320AIC31xx::dumpRegisters() {
	for (unsigned int i=0; i<sizeof(registerTable)/sizeof(registerTable[0]);i++) {
		readRegister(registerTable[i].reg);
	}
}
// High-level function to set the PLL
void TLV320AIC31xx::setPLL(uint8_t pll_p, uint8_t pll_r, uint8_t pll_j, uint16_t pll_d) {
	writeRegister(AIC31XX_PLLJ, pll_j);
	writeRegister(AIC31XX_PLLDLSB, pll_d & 0xff);
	writeRegister(AIC31XX_PLLDMSB, (pll_d>>8) & 0xff);
	modifyRegister(AIC31XX_PLLPR, AIC31XX_PLLPR_R_MASK, pll_r);
	modifyRegister(AIC31XX_PLLPR, AIC31XX_PLLPR_P_MASK, pll_p);
}

void TLV320AIC31xx::setPLLPower(bool power) {
	modifyRegister(AIC31XX_PLLPR, AIC31XX_PLLPR_POWER_MASK, power ? 1 : 0);
}

void TLV320AIC31xx::setCLKMUX(uint8_t pll_clkin, uint8_t codec_clkin) {
	// The source reference clock for the codec is chosen by programming the CODEC_CLKIN value on page 0 / register 4, bits D1–D0. 
	modifyRegister(AIC31XX_CLKMUX, AIC31XX_CODEC_CLKIN_MASK, codec_clkin);
	modifyRegister(AIC31XX_CLKMUX, AIC31XX_PLL_CLKIN_MASK, pll_clkin);
}

void TLV320AIC31xx::setNDACVal(uint8_t ndac) {
	modifyRegister(AIC31XX_NDAC, AIC31XX_NDAC_MASK, ndac);
}
void TLV320AIC31xx::setNDACPower(bool power) {
	modifyRegister(AIC31XX_NDAC, AIC31XX_NDAC_POWER_MASK, power);
}
void TLV320AIC31xx::setMDACVal(uint8_t mdac) {
	modifyRegister(AIC31XX_MDAC, AIC31XX_MDAC_MASK, mdac);
}
void TLV320AIC31xx::setMDACPower(bool power) {
	modifyRegister(AIC31XX_MDAC, AIC31XX_MDAC_POWER_MASK, power);
}
void TLV320AIC31xx::setDOSRVal(uint16_t dosr) {
	writeRegister(AIC31XX_DOSRLSB, dosr & 0xff);
	writeRegister(AIC31XX_DOSRMSB, (dosr>>8) & 0xff);
}

void TLV320AIC31xx::setNADCVal(uint8_t nadc) {
	modifyRegister(AIC31XX_NADC, AIC31XX_NADC_MASK, nadc);
}
void TLV320AIC31xx::setNADCPower(bool power) {
	modifyRegister(AIC31XX_NADC, AIC31XX_NADC_POWER_MASK, power);
}
void TLV320AIC31xx::setMADCVal(uint8_t madc) {
	modifyRegister(AIC31XX_MADC, AIC31XX_MADC_MASK, madc);
}
void TLV320AIC31xx::setMADCPower(bool power) {
	modifyRegister(AIC31XX_MADC, AIC31XX_MADC_POWER_MASK, power);
}
void TLV320AIC31xx::setAOSRVal(uint16_t aosr) {
	writeRegister(AIC31XX_AOSR, aosr);
}

void TLV320AIC31xx::setWordLength(uint8_t wordlength) {
	modifyRegister(AIC31XX_IFACE1, AIC31XX_IFACE1_DATALEN_MASK, wordlength);
}

void TLV320AIC31xx::setHSDetectInt1(bool enable) {
	modifyRegister(AIC31XX_INT1CTRL, AIC31XX_HSPLUGDET, enable);
	modifyRegister(AIC31XX_GPIO1, AIC31XX_GPIO1_FUNC_MASK, AIC31XX_GPIO1_INT1);
}

void TLV320AIC31xx::enableHeadsetDetect() {
	modifyRegister(AIC31XX_HSDETECT, AIC31XX_HSD_ENABLE, 0x01);
}

bool TLV320AIC31xx::isHeadsetDetected() {
	// modifyRegister(AIC31XX_HSDETECT, AIC31XX_HSD_TYPE_MASK, AIC31XX_HSD_HP); // fake Hs in simulation
	return (readRegister(AIC31XX_HSDETECT) & AIC31XX_HSD_TYPE_MASK) != AIC31XX_HSD_NONE;
}


// High-level function to enable and unmute the DAC and route it to the output mixer
void TLV320AIC31xx::enableDAC() {
	modifyRegister(AIC31XX_DACSETUP, AIC31XX_DAC_POWER_MASK,0x3);
	modifyRegister(AIC31XX_DACMIXERROUTE, AIC31XX_DACMIXERROUTE_DACL_MASK,0x1);
	modifyRegister(AIC31XX_DACMIXERROUTE, AIC31XX_DACMIXERROUTE_DACR_MASK,0x1);
}

void TLV320AIC31xx::setDACMute(bool mute) {
	if(mute) {
		modifyRegister(AIC31XX_DACMUTE, AIC31XX_DACMUTE_MASK,0x3);
	} else {
		// unmute
		modifyRegister(AIC31XX_DACMUTE, AIC31XX_DACMUTE_MASK,0x0);
	}
}

void TLV320AIC31xx::setDACVolume(float left_dB, float right_dB) {
    // Convert dB values to register format
    uint8_t leftRegValue = convertDACVolumeToRegisterValue(left_dB);
    uint8_t rightRegValue = convertDACVolumeToRegisterValue(right_dB);

    // Write to left and right DAC volume registers
    writeRegister(AIC31XX_LDACVOL, leftRegValue);  // Left DAC volume control
    writeRegister(AIC31XX_RDACVOL, rightRegValue); // Right DAC volume control
}

void TLV320AIC31xx::enableADC() {
	// power
	modifyRegister(AIC31XX_ADCSETUP, AIC31XX_ADC_POWER_MASK,0x1);
	// unmute
	modifyRegister(AIC31XX_ADCFGA, AIC31XX_ADC_MUTE_MASK,0x0);
}

// enable the headphone amplifier
void TLV320AIC31xx::enableHeadphoneAmp() {
	modifyRegister(AIC31XX_HPDRIVER, AIC31XX_HPD_POWER_MASK, 0x3);
}

void TLV320AIC31xx::setHeadphoneMute(bool mute) {
	modifyRegister(AIC31XX_HPLGAIN, AIC31XX_HPLGAIN_MUTE_MASK, mute ? 0x0 : 0x1);
	modifyRegister(AIC31XX_HPRGAIN, AIC31XX_HPRGAIN_MUTE_MASK, mute ? 0x0 : 0x1);
}

void TLV320AIC31xx::setHeadphoneGain(float left_dB, float right_dB) {
	modifyRegister(AIC31XX_HPLGAIN, AIC31XX_HPLGAIN_GAIN_MASK, convertPgaGainToRegisterValue(left_dB));
	modifyRegister(AIC31XX_HPRGAIN, AIC31XX_HPRGAIN_GAIN_MASK, convertPgaGainToRegisterValue(right_dB));
}

void TLV320AIC31xx::setHeadphoneVolume(float left_dB, float right_dB) {
	writeRegister(AIC31XX_LANALOGHPL,convertAnalogGainToRegisterValue(left_dB) );
	writeRegister(AIC31XX_RANALOGHPR,convertAnalogGainToRegisterValue(right_dB) );
}

void TLV320AIC31xx::setHeadphonePerformance(uint8_t level) {
	modifyRegister(AIC31XX_HPCONTROL, AIC31XX_HPCONTROL_PERFORMANCE_MASK, level);
}

void TLV320AIC31xx::setHeadphoneLineMode(bool line) {
	modifyRegister(AIC31XX_HPCONTROL, AIC31XX_HPCONTROL_HPL_LINE_MASK, line);
	modifyRegister(AIC31XX_HPCONTROL, AIC31XX_HPCONTROL_HPR_LINE_MASK, line);
}

void TLV320AIC31xx::enableSpeakerAmp() {
	modifyRegister(AIC31XX_SPKAMP, AIC3100_SPKAMP_POWER_MASK, 0x1);
	// TODO: also support other channel on codecs with stereo amp
}

void TLV320AIC31xx::setSpeakerMute(bool mute) {
	modifyRegister(AIC31XX_SPLGAIN, AIC3100_SPKLGAIN_MUTE_MASK, mute ? 0x0 : 0x1);
}

void TLV320AIC31xx::setSpeakerVolume(float left_dB) {
	writeRegister(AIC31XX_LANALOGSPL,convertAnalogGainToRegisterValue(left_dB) );
}

void TLV320AIC31xx::setSpeakerGain(float gaindb) {
	// Round the input to the nearest integer (to handle potential float inaccuracies)
	int roundedGain = static_cast<int>(gaindb + 0.5f);

	uint8_t gain = 0;
	switch (roundedGain) {
		case 12:
			gain = 0x1;
			break;
		case 18:
			gain = 0x2;
			break;
		case 24:
			gain = 0x3;
			break;
		default:
			gain = 0x0;
	}
	modifyRegister(AIC31XX_SPLGAIN, AIC3100_SPKLGAIN_GAIN_MASK, gain);
}

void TLV320AIC31xx::setMicPGAEnable(bool enable) {
	modifyRegister(AIC31XX_MICPGA, AIC31XX_MICPGA_ENABLE_MASK, enable ? 0x0 : 1);
}

void TLV320AIC31xx::setMicPGAGain(float gain) {
	modifyRegister(AIC31XX_MICPGA, AIC31XX_MICPGA_GAIN_MASK, convertMicPgaGainToRegisterValue(gain));
}

void TLV320AIC31xx::setADCGain(float adcGain) {
	writeRegister(AIC31XX_ADCVOL,convertADCVolumeToRegisterValue(adcGain) );
}

void TLV320AIC31xx::powerDown() {
	reset();
}

// High-level function to software-reset the codec (also turns off all components for power saving)
void TLV320AIC31xx::reset() {
    writeRegister(AIC31XX_RESET, 0x1);
}

