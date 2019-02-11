#pragma once
#include "main.h"
#include "jctc/misc/misc.hpp"
#include "jctc/odom/odom.hpp"
#include "jctc/pid/pid.hpp"

namespace jctc {
  class Chassis {
  private:
    odom::Tracker tracker;
    
  public:
    Chassis(odom::Tracker tracker) :
      tracker(tracker) {};
    virtual void driveVector(float forward, float turn);
  };
}
