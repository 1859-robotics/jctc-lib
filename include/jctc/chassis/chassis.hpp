#pragma once
#include "main.h"
#include "../misc/misc.hpp"
#include "../odom/odom.hpp"
#include "../odom/position.hpp"
#include "../pid/pid.hpp"

namespace jctc {
  class Chassis {
  private:
    odom::Tracker tracker;

    PID turnPid;
    PID distPid;

  public:
    Chassis(odom::Tracker tracker, PID turnPid, PID distPid) :
      tracker(tracker), turnPid(turnPid), distPid(distPid) {};

    virtual void driveVector(float forward, float turn) = 0;

    void driveTo(odom::Point target, float rotScalar, int timeout, float err);
    void turnToFace(float target, int timeout, float err);
    void turnToFace(odom::Point target, int timeout, float err);

    float angleToPoint(odom::Point target);
    float distanceToPoint(odom::Point target);
  };
}
