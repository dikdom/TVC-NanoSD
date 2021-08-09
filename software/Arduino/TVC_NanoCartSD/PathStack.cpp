#include "PathStack.h"

PathStack::PathStack() {
  pos = 0;
}

bool PathStack::push(byte startPos, byte endPos) {
  if(pos<MAX_PATHSTACK_SIZE) {
    stack[pos][0] = startPos;
    stack[pos][1] = endPos;
    pos++;
    return true;
  } else {
    return false;
  }
}

void PathStack::dropTop() {
  if(pos>0)
    pos--;
}

String PathStack::getAbsolutePath(String path) {
  String retVal;
  for(int i=0; i<pos; i++) {
    retVal+=path.substring(stack[i][0], stack[i][1]);
  }
  return retVal;
}

