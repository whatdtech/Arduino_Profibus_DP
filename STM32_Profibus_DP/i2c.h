#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

void I2C1_Init(void);
bool I2C1_WriteData(uint8_t dev_addr, uint8_t *data, uint16_t size);
bool I2C1_WriteMem(uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint16_t size);

#endif /* I2C_H */
