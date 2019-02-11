#pragma once
#include "main.h"
#include "jctc/misc/misc.hpp"
#include "jctc/odom/odom.hpp"
#include "jctc/pid/pid.hpp"

namespace jctc {
  class Chassis {
  private:
    odom::Tracker tracker;

    PID turnPid;
    PID distPid;

  public:
    Chassis(odom::Tracker tracker, PID turnPid, PID distPid) :
      tracker(tracker), turnPid(turnPid), distPid(distPid) {};

    virtual void driveVector(float forward, float turn);

    void driveTo(odom::Point target, float rotScalar, int timeout, float err);
    void turnToFace(float target, int timeout, float err);
    void turnToFace(odom::Point target, int timeout, float err);

  };
}
