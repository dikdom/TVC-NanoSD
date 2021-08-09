#ifndef CircularQueue_h
#define CircularQueue_h

#include "Arduino.h"



class CircularQueue {
  public: 
    CircularQueue(int n);
    ~CircularQueue();
    void push(byte data);
    void pushBlock(byte dataLength, byte *data);
    byte pop();
    bool isEmpty();
    bool isFull();
    void clear();
    byte getSize();
  private:
    byte *buffer;
//    volatile byte size;
    byte maxSize;
    volatile byte startPos;
    volatile byte endPos;
    byte getIncrementedPos(byte pos);
};
#endif // CircularQueue_h
