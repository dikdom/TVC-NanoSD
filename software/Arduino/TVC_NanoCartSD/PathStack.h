#ifndef PathStack_h
#define PathStack_h

#include "Arduino.h"

#define MAX_PATHSTACK_SIZE 8

class PathStack {
  public:
    PathStack();
    bool push(byte startPos, byte endPos);
    void dropTop();
    String getAbsolutePath(String path);
  private:
    byte stack[MAX_PATHSTACK_SIZE][2];
    byte pos = 0;
};

#endif