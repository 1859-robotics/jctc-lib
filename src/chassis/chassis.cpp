#include "jctc/chassis/chassis.hpp"

bool withinRange(float target, float current, float error) {
  return abs(target - current) < error;
}

namespace jctc {
  float Chassis::angleToPoint(odom::Point point) {
    odom::Position state = tracker.getPos();
    return (atan2(point.y - state.pos.y, point.x - state.pos.x)) - state.a;
  }

  float Chassis::distanceToPoint(odom::Point target) {
    odom::Position state = tracker.getPos();
    return odom::dist(target, state.pos);
  }

  //TODO: make timeout
  void Chassis::moveTo(odom::Point target, float rotScalar, int timeout, float err) {
    while(!withinErr(target, tracker.getPos().pos, err)) {
      const odom::Position state = tracker.getPos();

      odom::Point close = odom::closest({
        state.pos.x, state.pos.y                         // current
      }, { (float)cos(state.a), (float)sin(state.a)}, {  // head
        target.x, target.y                               // target
      });

      float angleErr = angleToPoint(close);
      if(std::isnan(angleErr)) angleErr = 0;

      float distanceErr = distanceToPoint(close);

      float angleVel = turnPid.calculate(angleErr, 0);
      float distanceVel = distPid.calculate(distanceErr, 0);

      driveVector(distanceVel, angleVel * rotScalar);

      pros::delay(20);
    }
    driveVector(0, 0);
  }

  void Chassis::moveToSimple(odom::Point target, int timeout) {
    turnToFace(target);
    moveFor(dist({ tracker.getPos().pos.x, tracker.getPos().pos.y}, target), timeout);
  }

  void Chassis::moveFor(float distIn, PIDConfig pid, float exit){
    odom::Point start = {
      tracker.getPos().pos.x,
      tracker.getPos().pos.y
    };

    distPid.reset();
    distPid.config(pid);

    std::uint32_t started = pros::millis();

    distPid.doPID(0, P_ERR, [=]() -> float {
      if((pros::millis() - started) > exit) return 0;
      return (fabs(distIn) - dist(start, { tracker.getPos().pos.x, tracker.getPos().pos.y }));
    }, [=](float output) -> void {
      driveVector(SGN(-distIn) * output, 0);
    });
    driveVector(0, 0);
  }

  void Chassis::moveFor(float distIn, float exit) {
    moveFor(distIn, { 8, 0, 0.1 }, exit);
  }

  void Chassis::turnToFace(float deg, int timeout, float err, PIDConfig config) {
    turnPid.reset();
    turnPid.config(config);

    if(withinRange(TODEG(tracker.getPos().a), deg, A_ERR)) return;

    turnPid.doPID(deg, err, [=]() -> float {
      return TODEG(tracker.getPos().a);
    }, [=](float output) -> void {
      driveVector(0, output);
    });

    driveVector(0, 0);
  }
  void Chassis::turnToFace(odom::Point target, int timeout, float err, PIDConfig config) {
    turnPid.reset();
    turnToFace(TODEG(atan2(target.y - tracker.getPos().pos.y, target.x - tracker.getPos().pos.x)), timeout, err, config);
  }

}
