#pragma once
#include "main.h"
#include "../misc/misc.hpp"
#include "../odom/odom.hpp"
#include "../odom/position.hpp"
#include "../pid/pid.hpp"

#ifndef P_ERR
#define P_ERR 3
#endif

#ifndef A_ERR
#define A_ERR 3
#endif

namespace jctc {
  class Chassis {
  private:
    odom::Tracker tracker;

    PID turnPid;
    PID distPid;

  public:
    Chassis(odom::Tracker tracker, PID turnPid, PID distPid) :
      tracker(tracker), turnPid(turnPid), distPid(distPid) {};

    virtual void driveVector(float forward, float turn) ;

    void moveTo(odom::Point target, float rotScalar, int timeout, float err);
    void moveToSimple(odom::Point target, int timeout = 5000);

    void moveFor(float distIn, float exit = 5000);
    void moveFor(float distIn, pid::PIDConfig pid, float exit = 5000);

    void turnToFace(float target, int timeout, float err);
    void turnToFace(odom::Point target, int timeout, float err);

    float angleToPoint(odom::Point target);
    float distanceToPoint(odom::Point target);
  };
}
