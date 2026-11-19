#include "lps22hb.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include <stdio.h>
#include <math.h>
#include "param.h"
#include "tools.h"

extern CONFIG config;

LPS22HB::LPS22HB() {
    lastTimestampMicros = 0;
    groundPressure = 0;
}

bool LPS22HB::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value}; // Korrigiert: Jetzt ein echtes Array!
    return (i2c_write_blocking(i2c1, LPS22HB_I2C_ADDRESS, data, 2, false) >= 0);
}

bool LPS22HB::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    if (i2c_write_blocking(i2c1, LPS22HB_I2C_ADDRESS, &reg, 1, true) < 0) return false;
    return (i2c_read_blocking(i2c1, LPS22HB_I2C_ADDRESS, buffer, length, false) >= 0);
}

void LPS22HB::begin() 
{ 
    baroInstalled = false;
    tryAddress(0x5C); 
    if (baroInstalled == false) {
        tryAddress(0x5D); 
    }
}

void LPS22HB::tryAddress(uint8_t address)
{
    uint8_t writeCmd[2];
    uint8_t readValue;
    uint8_t regToRead; 
    int i2cError;
    baroInstalled = false;

    if (config.pinScl == 255 or config.pinSda == 255) return; 

    _address = address;
    #ifdef DEBUG  
    printf("Trying to detect LPS22HB sensor at I2C Addr=%X\n", _address);
    #endif

    // 1. Device ID (WHO_AM_I) prüfen (muss 0xB1 sein)
    regToRead = LPS22HB_WHO_AM_I; 
    i2cError = i2c_write_timeout_us(i2c1, _address, &regToRead, 1, false, 1000);
    if (i2cError < 0) {
        #ifdef DEBUG
        printf("Write error for LPS22HB: %i\n", i2cError);
        #endif
        return; 
    }
    i2cError = i2c_read_timeout_us(i2c1, _address, &readValue, 1, false, 1500);
    if (i2cError < 0) {
        #ifdef DEBUG
        printf("Read error for LPS22HB: %i\n", i2cError);
        #endif
        return;
    }     
    if (readValue != LPS22HB_WHO_AM_I_VALUE) {
        #ifdef DEBUG
        printf("LPS22HB has wrong device id\n");
        #endif
        return;
    }

    // 2. Sensor konfigurieren (CTRL_REG1 = 0x58)
    writeCmd[0] = LPS22HB_CTRL_REG1;
    writeCmd[1] = 0x58;
    i2cError = i2c_write_timeout_us(i2c1, _address, &writeCmd[0], 2, false, 1000);
    if (i2cError < 0) {
        #ifdef DEBUG
        printf("Write error for LPS22HB config: %i\n", i2cError);
        #endif
        return;  
    }

    sleep_ms(100); 

    // 3. Startplatz-Nullung
    int32_t sumPressure = 0;
    int validReadings = 0;
    uint8_t readBuffer[3];

    for (int i = 0; i < 20; i++) {
        regToRead = LPS22HB_PRESS_OUT_XL;
        if (i2c_write_timeout_us(i2c1, _address, &regToRead, 1, false, 1000) >= 0) {
            if (i2c_read_timeout_us(i2c1, _address, &readBuffer[0], 3, false, 1500) >= 0) {
                int32_t p_raw = (int32_t)readBuffer[0] | ((int32_t)readBuffer[1] << 8) | ((int32_t)readBuffer[2] << 16);
                if (p_raw & 0x00800000) p_raw |= 0xFF000000;
                sumPressure += (p_raw * 25) / 1024;
                validReadings++;
            }
        }
        sleep_ms(20); 
    }

    if (validReadings > 0) {
        groundPressure = sumPressure / validReadings;
    } else {
        groundPressure = 101325; 
    }

    lastTimestampMicros = time_us_32();

    #ifdef DEBUG  
    printf("LPS22HB sensor is successfully detected\n");  
    #endif
    baroInstalled = true; 
}



/*bool LPS22HB::begin() {
    uint8_t whoAmI = 0;
    if (!readRegisters(LPS22HB_WHO_AM_I, &whoAmI, 1)) return false;
    if (whoAmI != LPS22HB_WHO_AM_I_VALUE) return false;

    if (!writeRegister(LPS22HB_CTRL_REG1, 0x58)) return false;
    sleep_ms(100); 

    baroInstalled = true;
    lastTimestampMicros = time_us_32();

    // Startplatz-Nullung
    int32_t sumPressure = 0;
    int validReadings = 0;
    uint8_t readBuffer[3]; // Korrigiert: Lokales Array für den Lese-Puffer

    for (int i = 0; i < 20; i++) {
        if (readRegisters(LPS22HB_PRESS_OUT_XL, readBuffer, 3)) {
            int32_t p_raw = (int32_t)readBuffer[0] | ((int32_t)readBuffer[1] << 8) | ((int32_t)readBuffer[2] << 16);
            if (p_raw & 0x00800000) p_raw |= 0xFF000000;
            sumPressure += (p_raw * 25) / 1024;
            validReadings++;
        }
        sleep_ms(20); 
    }

    if (validReadings > 0) {
        groundPressure = sumPressure / validReadings;
    } else {
        groundPressure = 101325; 
    }

    return true;
}

int LPS22HB::getAltitude() {
    uint8_t readBuffer[3]; // Korrigiert: Lokales Array für den Lese-Puffer
    if (!readRegisters(LPS22HB_PRESS_OUT_XL, readBuffer, 3)) return -1; 

    int32_t p_raw = (int32_t)readBuffer[0] | ((int32_t)readBuffer[1] << 8) | ((int32_t)readBuffer[2] << 16);
    if (p_raw & 0x00800000) p_raw |= 0xFF000000;

    rawPressure = (p_raw * 25) / 1024; 

    uint32_t now = time_us_32();
    altIntervalMicros = now - lastTimestampMicros;
    lastTimestampMicros = now;

    // Relative Höhenberechnung in Zentimetern (cm) zum eingemessenen Bodendruck
    float relAltitudeMeters = 44330.0f * (1.0f - powf((float)rawPressure / (float)groundPressure, 0.190295f));
    altitudeCm = (int32_t)(relAltitudeMeters * 100.0f);

    return 0; 
}*/
int LPS22HB::getAltitude() {
    // 1. Absturzschutz: Sofort abbrechen, wenn Sensor beim Start nicht gefunden wurde
    if (!baroInstalled) return -1;

    // 2. Timing-Schutz: Maximal alle 20ms (50 Hz) abfragen, genau wie beim BMP280
    uint32_t now = microsRp();
    if ((now - lastTimestampMicros) < 20000) return -1;

    uint8_t readBuffer[3];
    if (!readRegisters(LPS22HB_PRESS_OUT_XL, readBuffer, 3)) return -1; 

    int32_t p_raw = (int32_t)readBuffer[0] | ((int32_t)readBuffer[1] << 8) | ((int32_t)readBuffer[2] << 16);
    if (p_raw & 0x00800000) p_raw |= 0xFF000000;

    rawPressure = (p_raw * 25) / 1024; 

    altIntervalMicros = now - lastTimestampMicros;
    lastTimestampMicros = now;

    // 3. Einheitliche Höhenberechnung wie BMP280 (oXs übernimmt das Nullen selbst)
    altitudeCm = 4433000.0 * (1.0 - pow((double)rawPressure / 101325.0, 0.1903));

    return 0; 
}
