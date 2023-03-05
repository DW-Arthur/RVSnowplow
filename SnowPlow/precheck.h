#ifndef PRECHECK_H
#define PRECHECK_H

#include "Arduino.h"

#define ERROR_LED 13


class Precheck{
  public:
  static void init();
  static void fail();

};

#endif