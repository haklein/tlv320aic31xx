#ifndef TLV320AIC31XX_CODEC_H
#define TLV320AIC31XX_CODEC_H

#include <stdint.h>
#include "tlv320aic31xx_regs.h" // Include header file with register definitions and macros

#define TLV320AIC31XX_I2C_ADDRESS 0x18 // Default I2C address for TLV320AIC31xx

class TLV320AIC31xx {
private:
    static const uint8_t PAGE_CTRL_REGISTER = 0x00;
    // helper to set the active page
    void setPage(uint8_t page);

    // extract page and register address from AIC31XX_REG macro
    uint8_t getPage(uint16_t reg);
    uint8_t getRegister(uint16_t reg);

    // helper to convert dB from float to register values
    uint8_t convertDACVolumeToRegisterValue(float dB); // Table 7-106
    uint8_t convertADCVolumeToRegisterValue(float dB); // Table 7-124 
    uint8_t convertAnalogGainToRegisterValue(float dB); // Table 7-38
    uint8_t convertPgaGainToRegisterValue(float dB); // Table 7-157 
    uint8_t convertMicPgaGainToRegisterValue(float dB); // Table 7-163

    uint8_t current_page = 0;
public:
    // Constructor
    TLV320AIC31xx();

    // Initialize the codec
    void initialize();

    // Write a single register
    void writeRegister(uint16_t reg, uint8_t value);

    // Read a single register
    uint8_t readRegister(uint16_t reg);

    // Read, modify, and write back specific bits in a register
    // auto-shifts value to the mask
    void modifyRegister(uint16_t reg, uint8_t mask, uint8_t value);

    // Configure the PLL clocks
    void configureClocks(uint8_t pll_p, uint8_t pll_r, uint8_t pll_j, uint16_t pll_d);

    void setCLKMUX(uint8_t pll_clkin, uint8_t codec_clkin);

    void setPLLPower(bool power);

    void setNDACVal(uint8_t ndac);
    void setNDACPower(bool power);
    void setMDACVal(uint8_t mdac);
    void setMDACPower(bool power);
    void setDOSRVal(uint16_t dosr);

    void setNADCVal(uint8_t nadc);
    void setNADCPower(bool power);
    void setMADCVal(uint8_t madc);
    void setMADCPower(bool power);
    void setAOSRVal(uint16_t aosr);

    void setWordLength(uint8_t wordlength);

    void setHSDetectInt1(bool enable);

    // Enable and unmute DAC
    void enableDAC();

    void setDACVolume(float left_dB, float right_dB);

    void enableADC();
    void setADCGain(float adcGain);

    // Enable headphone amplifier
    void enableHeadphoneAmp();
    void setHeadphoneMute(bool mute);
    void setHeadphoneGain(float left_dB, float right_dB);
    void setHeadphoneVolume(float left_dB, float right_dB);
    void setHeadphonePerformance(uint8_t level);
    void setHeadphoneLineMode(bool line);

    void enableSpeakerAmp();
    void setSpeakerMute(bool mute);
    void setSpeakerGain(float gaindB);
    void setSpeakerVolume(float left_dB);

    void enableHeadsetDetect();
    bool isHeadsetDetected();

    void setMicPGAEnable(bool enable);
    void setMicPGAGain(float gain);

    // Power down the codec
    void powerDown();

    // Reset the codec
    void reset();

};

#endif // TLV320AIC31XX_CODEC_H
