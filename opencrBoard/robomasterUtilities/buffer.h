#ifndef BUFFER_H
#define BUFFER_H

#include "commands.h"

typedef enum {
    BUFFER_READY,     
    BUFFER_EMPTY,    
    BUFFER_FULL   
} BufferStatus;

class CircularBuffer
{
private:
    Command* buffer;
    int head;
    int tail;
    int capacity;

public:
    CircularBuffer(int capacity);
    ~CircularBuffer();

    BufferStatus push(Command element);
    BufferStatus pop(Command& data);
};


#endif  // BUFFER_H