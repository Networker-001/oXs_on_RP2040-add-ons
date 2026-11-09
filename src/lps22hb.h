#ifndef _LPS22HB_H_
#define _LPS22HB_H_

#include <stdint.h>
#include "hardware/i2c.h"

#define LPS22HB_I2C_ADDRESS     0x5C
#define LPS22HB_WHO_AM_I        0x0F
#define LPS22HB_CTRL_REG1       0x10
#define LPS22HB_PRESS_OUT_XL    0x28
#define LPS22HB_PRESS_OUT_L     0x29
#define LPS22HB_PRESS_OUT_H     0x2A
#define LPS22HB_WHO_AM_I_VALUE  0xB1

class LPS22HB {
public:
    LPS22HB();
    bool begin();
    int getAltitude(); 
    bool baroInstalled = false;
    int32_t altitudeCm;         // Das braucht deine main.cpp!
    uint32_t altIntervalMicros; 
    int64_t rawPressure ;  // in 1/10000 mBar so = Pa 
    uint8_t _address = 0x5C;

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);
    
    int32_t groundPressure;     // Für das automatische Nullen beim Start
    uint32_t lastTimestampMicros;
};

#endif // _LPS22HB_H_
