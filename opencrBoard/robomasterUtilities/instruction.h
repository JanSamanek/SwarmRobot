#ifndef INSTRUCTION_H
#define INSTRUCTION_H

#include <stdint.h>

typedef struct {
    uint16_t speedX;
    uint16_t speedY;
    uint16_t rotation;
    int16_t gimballYaw;
    int16_t gimballRoll;
} Instruction;

#endif // INSTRUCTION_H

