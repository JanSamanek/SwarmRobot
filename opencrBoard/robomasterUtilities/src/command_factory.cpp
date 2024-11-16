#include "command_factory.h"
#include "checksum.h"


static void updateMoveCommand(Command&  command, const Instruction& instruction)
{
    command.data[11] = instruction.speedY & 0xFF;
    command.data[12] = ((instruction.speedX << 3) | (instruction.speedY >> 8)) & 0x07;
    command.data[13] = (instruction.speedX >> 5) & 0x3F;

    command.data[16] = (instruction.rotation << 4) | 0x08;
    command.data[17] = (instruction.rotation >> 4) & 0xFF;

    command.data[19] = 0x02 | ((instruction.rotation << 2) & 0xFF);
    command.data[20] = (instruction.rotation >> 6) & 0xFF;
}

static void updateGimballCommand(Command&  command, const Instruction& instruction)
{
    command.data[13] = instruction.gimballRoll & 0xFF;
    command.data[14] = (instruction.gimballRoll >> 8) & 0xFF;
    command.data[15] = instruction.gimballYaw & 0xFF;
    command.data[16] = (instruction.gimballYaw >> 8) & 0xFF;
}

static void updateCommandCounter(Command&  command, int command_counter)
{
    command.data[6] = command_counter & 0xFF;
    command.data[7] = (command_counter >> 8) & 0xFF;
}

static void updateCrc(Command& command, uint32_t command_length)
{
    setCRC8(command.data, 3);
    appendCRC16CheckSum(command.data, command_length);
}

static void preprocessCommand(Command& command, const Instruction& instruction, int command_counter)
{
     if(command.type == GIMBALL_COMMAND)
     {
        updateGimballCommand(command, instruction);
     }
            
     if(command.type == MOVE_COMMAND)
     {
        updateMoveCommand(command, instruction);
     }

    updateCommandCounter(command, command_counter);
    updateCrc(command, command.length);
}


uint32_t CommandFactory::commandCounter = 0;

CommandFactory::CommandFactory()
{
    command1 = {
    .data = new uint8_t[13]{0x55, 0x0D, 0x04, 0xFF, 0x0A, 
                        0xFF, 0xFF, 0xFF, 0x40, 0x00, 
                        0x01, 0xFF, 0xFF},
    .length = 13,
    .type = COMMAND_1
    };

    command2 = {
        .data = new uint8_t[14]{0x55, 0x0E, 0x04, 0xFF, 0x09, 
                            0x03, 0xFF, 0xFF, 0xA0, 0x48, 
                            0x08, 0x01, 0xFF, 0xFF},
        .length = 14,
        .type = COMMAND_2
    };

    command3 = {
        .data = new uint8_t[15]{0x55, 0x0F, 0x04, 0xFF, 0xF1, 
                            0xC3, 0xFF, 0xFF, 0x00, 0x0A, 
                            0x53, 0x32, 0x00, 0xFF, 0xFF},
        .length = 15,
        .type = COMMAND_3
    };

    command4 = {
        .data = new uint8_t[18]{0x55, 0x12, 0x04, 0xFF, 0xF1, 
                            0xC3, 0xFF, 0xFF, 0x40, 0x00, 
                            0x58, 0x03, 0x92, 0x06, 0x02, 
                            0x00, 0xFF, 0xFF},
        .length = 18,
        .type = COMMAND_4
    };

    command5 = {
        .data = new uint8_t[73]{0x55, 0x49, 0x04, 0xFF, 0x49, 
                            0x03, 0xFF, 0xFF, 0x00, 0x3F, 
                            0x70, 0xB4, 0x0F, 0x66, 0x03, 
                            0x00, 0x00, 0xD3, 0x03, 0x3D, 
                            0x08, 0x10, 0x00, 0x08, 0x00, 
                            0x08, 0x00, 0x08, 0x00, 0x08, 
                            0x00, 0x08, 0x00, 0x08, 0x00, 
                            0x00, 0x00, 0x00, 0x00, 0x00, 
                            0x00, 0x00, 0x00, 0x00, 0x00, 
                            0x00, 0x00, 0x00, 0x7E, 0x0E, 
                            0xF3, 0x0B, 0xD9, 0x07, 0x0E, 
                            0x07, 0x3D, 0x07, 0x6A, 0x08, 
                            0x62, 0x0A, 0x05, 0x0B, 0xD6, 
                            0x0B, 0xFF, 0xFF},
        .length = 73,
        .type = COMMAND_5
    };

    gimballCommand = {
        .data = new uint8_t[20]{0x55, 0x14, 0x04, 0xFF, 0x09, 
                            0x04, 0xFF, 0xFF, 0x00, 0x04, 
                            0x69, 0x08, 0x05, 0x00, 0x00, 
                            0x00, 0x00, 0x6C, 0xFF, 0xFF},
        .length = 20,
        .type = GIMBALL_COMMAND
    };

    moveCommand = {
        .data = new uint8_t[27]{0x55, 0x1B, 0x04, 0xFF, 0x09, 
                            0xC3, 0xFF, 0xFF, 0x00, 0x3F, 
                            0x60, 0x00, 0x04, 0x20, 0x00, 
                            0x01, 0x08, 0x40, 0x00, 0x02, 
                            0x10, 0x04, 0x0C, 0x00, 0x04, 
                            0xFF, 0xFF},
        .length = 27,
        .type = MOVE_COMMAND
    };

    blasterCommand1 = {
        .data = new uint8_t[14]{0x55, 0x0E, 0x04, 0xFF, 0x09, 
                            0x17, 0xFF, 0xFF, 0x00, 0x3F, 
                            0x51, 0x01, 0xFF, 0xFF},
        .length = 14,
        .type = BLASTER_COMMAND_1
    };

    blasterCommand2 = {
        .data = new uint8_t[22]{0x55, 0x16, 0x04, 0xFF, 0x09, 
                            0x17, 0xFF, 0xFF, 0x00, 0x3F, 
                            0x55, 0x73, 0x00, 0xFF, 0x00, 
                            0x01, 0x28, 0x00, 0x00, 0x00, 
                            0xFF, 0xFF},
        .length = 22,
        .type = BLASTER_COMMAND_2
    };

    initFreeModeCommand = {
        .data = new uint8_t[14]{0x55, 0x0E, 0x04, 0xFF, 0x09, 
                            0xC3, 0xFF, 0xFF, 0x40, 0x3F, 
                            0x3F, 0x01, 0xFF, 0xFF},
        .length = 14,
        .type = INIT_FREE_MODE_COMMAND
    };

    initChasisAccelerationCommand = {
        .data = new uint8_t[14]{0x55, 0x0E, 0x04, 0xFF, 0x09, 
                            0xC3, 0xFF, 0xFF, 0x40, 0x3F, 
                            0x28, 0x02, 0xFF, 0xFF},
        .length = 14,
        .type = INIT_CHASSIS_ACCELERATION_COMMAND
    };

    initCommand1 = {
        .data = new uint8_t[21]{0x55, 0x15, 0x04, 0xFF, 0xF1, 
                            0xC3, 0x00, 0x00, 0x00, 0x03, 
                            0xD7, 0x01, 0x07, 0x00, 0x02, 
                            0x00, 0x00, 0x00, 0x00, 0xFF, 
                            0xFF},
        .length = 21,
        .type = INIT_COMMAND_1
    };

    initCommand2 = {
        .data = new uint8_t[18]{0x55, 0x12, 0x04, 0xFF, 0x09, 
                            0x03, 0x01, 0x00, 0x40, 0x48, 
                            0x01, 0x09, 0x00, 0x00, 0x00, 
                            0x03, 0xFF, 0xFF},
        .length = 18,
        .type = INIT_COMMAND_2
    };

    initCommand3 = {
        .data = new uint8_t[28]{0x55, 0x1C, 0x04, 0xFF, 0x09, 
                            0x03, 0x02, 0x00, 0x40, 0x48, 
                            0x03, 0x09, 0x00, 0x03, 0x00, 
                            0x01, 0xFB, 0xDC, 0xF5, 0xD7, 
                            0x03, 0x00, 0x02, 0x00, 0x01, 
                            0x00, 0xFF, 0xFF},
        .length = 28,
        .type = INIT_COMMAND_3
    };

    initCommand4 = {
        .data = new uint8_t[18]{0x55, 0x12, 0x04, 0xFF, 0x09, 
                            0x03, 0x03, 0x00, 0x40, 0x48, 
                            0x01, 0x09, 0x00, 0x00, 0x00, 
                            0x03, 0xFF, 0xFF},
        .length = 18,
        .type = INIT_COMMAND_4
    };

    initCommand5 = {
        .data = new uint8_t[36]{0x55, 0x24, 0x04, 0xFF, 0x09, 
                            0x03, 0x04, 0x00, 0x40, 0x48, 
                            0x03, 0x09, 0x01, 0x03, 0x00, 
                            0x02, 0xA7, 0x02, 0x29, 0x88, 
                            0x03, 0x00, 0x02, 0x00, 0x66, 
                            0x3E, 0x3E, 0x4C, 0x03, 0x00, 
                            0x02, 0x00, 0x32, 0x00, 0xFF, 
                            0xFF},
        .length = 36,
        .type = INIT_COMMAND_5
    };
}

CommandFactory::~CommandFactory()
{
    delete[] command1.data;
    delete[] command2.data;
    delete[] command3.data;
    delete[] command4.data;
    delete[] command5.data;
    delete[] gimballCommand.data;
    delete[] moveCommand.data;
    delete[] blasterCommand1.data;
    delete[] blasterCommand2.data;
    delete[] initFreeModeCommand.data;
    delete[] initChasisAccelerationCommand.data;
    delete[] initCommand1.data;
    delete[] initCommand2.data;
    delete[] initCommand3.data;
    delete[] initCommand4.data;
    delete[] initCommand5.data;
}

Command CommandFactory::buildCommand(CommandType type, const Instruction& instruction) const
{
    Command resultCommand;

    switch (type) {
        case COMMAND_1:
            resultCommand = command1;
            break;

        case COMMAND_2:
            resultCommand = command2;
            break;

        case COMMAND_3:
            resultCommand = command3;
            break;

        case COMMAND_4:
            resultCommand = command4;
            break;

        case COMMAND_5:
            resultCommand = command5;
            break;

        case GIMBALL_COMMAND:
            resultCommand = gimballCommand;
            break;

        case MOVE_COMMAND:
            resultCommand = moveCommand;
            break;

        case BLASTER_COMMAND_1:
            resultCommand = blasterCommand1;
            break;

        case BLASTER_COMMAND_2:
            resultCommand = blasterCommand2;
            break;

        case INIT_FREE_MODE_COMMAND:
            resultCommand = initFreeModeCommand;
            break;

        case INIT_CHASSIS_ACCELERATION_COMMAND:
            resultCommand = initChasisAccelerationCommand;
            break;

        case INIT_COMMAND_1:
            resultCommand = initCommand1;
            break;

        case INIT_COMMAND_2:
            resultCommand = initCommand2;
            break;

        case INIT_COMMAND_3:
            resultCommand = initCommand3;
            break;

        case INIT_COMMAND_4:
            resultCommand = initCommand4;
            break;

        case INIT_COMMAND_5:
            resultCommand = initCommand5;
            break;
    }


    preprocessCommand(resultCommand, instruction, commandCounter++);
    return resultCommand;
}
