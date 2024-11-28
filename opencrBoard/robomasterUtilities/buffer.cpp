#include "buffer.h"                                 
#include <vector>

CircularBuffer::CircularBuffer(int capacity)
:
capacity(capacity),
head(0),
tail(0)
{
    buffer = new Command[capacity];
}

CircularBuffer::~CircularBuffer()
{
    delete[] buffer;
}

BufferStatus CircularBuffer::push(Command element)
{
    int next;

    next = head + 1;      
    if (next >= capacity)
        next = 0;

    if (next == tail)         
        return BUFFER_FULL;

    buffer[head] = element;    
    
    head = next;            

    return BUFFER_READY;          
}

BufferStatus CircularBuffer::pop(Command& command)
{
    int next;

    if (head == tail)   
        return BUFFER_EMPTY;

    next = tail + 1;          
    if(next >= capacity)
        next = 0;

    command = buffer[tail];  
    tail = next;             
    return BUFFER_READY;           
}
