#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#include "tlv320aic31xx_codec.h"

TLV320AIC31xx codec(&Wire);

// Arduino Setup
void setup(void) {  
    // Open Serial 
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    codec.initialize();
    sleep(1);
    codec.setWordLength(AIC31XX_WORD_LEN_16BITS);
    codec.setCLKMUX(AIC31XX_PLL_CLKIN_BCLK, AIC31XX_CODEC_CLKIN_PLL);
    codec.configureClocks(1, 2, 32, 0); // uint8_t pll_p, uint8_t pll_r, uint8_t pll_j, uint16_t pll_d
    codec.setNDACVal(8);
    codec.setNDACPower(true);
    codec.setMDACVal(2);
    codec.setMDACPower(true);
    codec.setNADCVal(8);
    codec.setNADCPower(true);
    codec.setMADCVal(2);
    codec.setMADCPower(true);
    codec.setPLLPower(1);
    codec.enableHeadsetDetect();
    codec.setHSDetectInt1(true);
    codec.enableDAC();
    codec.setDACMute(false);
    codec.setDACVolume(20.0f,20.0f);
    codec.enableADC();
    codec.setADCGain(-12.0f);
    codec.enableHeadphoneAmp();
    codec.setHeadphoneMute(false); // unmute
    codec.setHeadphoneVolume(-5.0f,-5.0f); // unmute
    codec.setHeadphoneGain(8.0f,8.0f);
    codec.enableSpeakerAmp();
    codec.setSpeakerMute(false); // unmute class d speaker amp
    codec.setSpeakerGain(12.0f); // valid db: 6, 12, 18, 24
    codec.setSpeakerVolume(-5.0f); // unmute
    codec.dumpRegisters();
    //std::cout <<  "Is HS detected: " << codec.isHeadsetDetected() << std::endl << std::endl;
}

void loop() {
  sleep(1);
    Serial.print("HS Detect: ");
    Serial.println(codec.isHeadsetDetected());
}
#else

#include <iostream>
#include "tlv320aic31xx_codec.h"

int main() {
    TLV320AIC31xx codec;
    codec.initialize();
    codec.setWordLength(AIC31XX_WORD_LEN_16BITS);
    codec.setCLKMUX(AIC31XX_PLL_CLKIN_BCLK, AIC31XX_CODEC_CLKIN_PLL);
    std::cout << "configureClocks(128,2,32,0)" << std::endl;
    codec.configureClocks(1, 2, 32, 0); // uint8_t pll_p, uint8_t pll_r, uint8_t pll_j, uint16_t pll_d
    codec.setNDACVal(8);
    codec.setNDACPower(true);
    codec.setMDACVal(2);
    codec.setMDACPower(true);
    codec.setNADCVal(8);
    codec.setNADCPower(true);
    codec.setMADCVal(2);
    codec.setMADCPower(true);
    codec.setPLLPower(1);
    codec.enableHeadsetDetect();
    codec.setHSDetectInt1(true);
    codec.enableDAC();
    codec.setDACMute(false);
    codec.setDACVolume(20.0f,20.0f);
    codec.enableADC();
    codec.setADCGain(-12.0f);
    codec.enableHeadphoneAmp();
    codec.setHeadphoneMute(false); // unmute
    codec.setHeadphoneVolume(-5.0f,-5.0f); // unmute
    codec.setHeadphoneGain(8.0f,8.0f);
    codec.enableSpeakerAmp();
    codec.setSpeakerMute(false); // unmute class d speaker amp
    codec.setSpeakerGain(18.0f); // valid db: 6, 12, 18, 24
    std::cout <<  "Is HS detected: " << codec.isHeadsetDetected() << std::endl << std::endl;
    std::cout << "REGISTER DUMP:" << std::endl << std::endl;
    codec.dumpRegisters();
}
#endif
