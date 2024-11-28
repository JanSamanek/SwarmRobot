#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>

typedef enum
{
    COMMAND_1,
    COMMAND_2,
    COMMAND_3,
    COMMAND_4,
    COMMAND_5,
    GIMBALL_COMMAND,
    MOVE_COMMAND,
    BLASTER_COMMAND_1,
    BLASTER_COMMAND_2,
    INIT_FREE_MODE_COMMAND,
    INIT_CHASSIS_ACCELERATION_COMMAND,
    INIT_COMMAND_1,
    INIT_COMMAND_2,
    INIT_COMMAND_3,
    INIT_COMMAND_4,
    INIT_COMMAND_5
} CommandType;

typedef struct {
    uint8_t* data;   
    uint16_t length;   
    CommandType type;    
} Command;


void sendCommand(Command command);

#endif // COMMANDS_H