#include "lps22hb.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include <stdio.h>
#include <math.h>

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

bool LPS22HB::begin() {
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
}
