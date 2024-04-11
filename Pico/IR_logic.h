#ifndef IR_logic_H
#define IR_logic_H

#include <Arduino.h>

#include "IR.h"

class IR_logic {
  public:
    IR_logic();
    String setI2CAddress(int address);
  private:
    IR ir;
};





