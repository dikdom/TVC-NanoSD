#include "CircularQueue.h"

CircularQueue::CircularQueue(int n) {
  maxSize = n+1;
  buffer = (byte *)malloc(maxSize);
  startPos = 0;
  endPos = 0;
}

CircularQueue::~CircularQueue() {
  free(buffer);
}

byte CircularQueue::getIncrementedPos(byte pos) {
  byte incPos = pos+1;
  if(incPos==maxSize)
    incPos = 0;
  return incPos;
}

void CircularQueue::push(byte data) {
  byte nextEndPos = getIncrementedPos(endPos);
  while(nextEndPos == startPos) {}
  buffer[endPos] = data;
  endPos = nextEndPos;
}

byte CircularQueue::pop() {
  while(startPos == endPos) {}
  byte retVal = buffer[startPos];
  startPos = getIncrementedPos(startPos);
  return retVal;
}

bool CircularQueue::isEmpty() {
  return startPos == endPos;
}

bool CircularQueue::isFull() {
  byte e = endPos;
  byte s = startPos;
  return s==e+1 ||
          ((e == maxSize-1) && (s == 0));
}

byte CircularQueue::getSize() {
  byte e = endPos;
  byte s = startPos;
  if(e>=s)
    return e-s;
  else
    return ((int)e + maxSize) - s;
  
}

void CircularQueue::pushBlock(byte dataLength, byte *data) {
  while(getSize()+dataLength >= maxSize) {}
  byte copyLength = min(dataLength, maxSize - endPos);
  memcpy(&buffer[endPos], data, copyLength);
  if(copyLength != dataLength) {
    memcpy(buffer, &data[copyLength], dataLength - copyLength);
    endPos = dataLength - copyLength;
  } else {
    if(dataLength+endPos==maxSize)
      endPos = 0;
    else
      endPos += dataLength;
  }
}

void CircularQueue::clear() {
  startPos = endPos = 0;
}