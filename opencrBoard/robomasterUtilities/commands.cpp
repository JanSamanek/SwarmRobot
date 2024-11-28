#include "commands.h"
#include <CAN.h>

void sendCommand(Command command)
{
     can_message_t message;
    message.id = 0x201;
    message.format = CAN_STD_FORMAT;

    uint32_t commandLen = command.length;
    // Serial.printf("message (%i): \n", commandLen);
    uint16_t commandDataIndex = 0;
    while (commandLen > 0)
    {
        message.length = 8;
        for (int i = 0; i < 8; i++)
        {
            if (!(commandLen > 0))
            {
                message.length = i;
                break;
            }
            message.data[i] = command.data[commandDataIndex++];
            commandLen--;
            // Serial.printf("0x%02X ", message.data[i]);
        }
        CanBus.writeMessage(&message);
    }
    // Serial.println();
}