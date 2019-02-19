#include "jctc/chassis/chassis.hpp"

namespace jctc {
  float Chassis::angleToPoint(odom::Point point) {
    odom::Position state = tracker.getPos();
    return (atan2(point.x - state.pos.x, point.y - state.pos.y)) - state.a;
  }
  float Chassis::distanceToPoint(odom::Point target) {
    odom::Position state = tracker.getPos();
    return odom::dist(target, state.pos);
  }

  //TODO: make timeout
  void Chassis::driveTo(odom::Point target, float rotScalar, int timeout, float err) {
    while(!withinErr(target, tracker.getPos().pos, err)) {
      const odom::Position state = tracker.getPos();
      float tA = rollPI(atan2(target.y - state.pos.y, target.x - state.pos.x) - state.a);

      odom::Point close = odom::closest({
        state.pos.x, state.pos.y                         // current
      }, { (float)cos(state.a), (float)sin(state.a)}, {  // head
        target.x, target.y                               // target
      });

      float angleErr = angleToPoint(target);
      float distanceErr = distanceToPoint(target);

      float angleVel = turnPid.calculate(0, angleErr);
      float distanceVel = distPid.calculate(0, distanceErr);

      driveVector(distanceVel, angleVel * rotScalar);

      pros::delay(20);
    }
    driveVector(0, 0);
  }

  void Chassis::moveToSimple(odom::Point target, int timeout) {
    turnToFace(target, 60);
    moveFor(dist(posTracker.x, posTracker.y, target.x, target.y), timeout);
  }

  void moveFor(float dist, pid::PIDConfig pid, float exit = 5000){
    w::odom::Point start = {
      posTracker.x,
      posTracker.y
    };

    mainPID.reset();
    mainPID.config(pid);

    std::uint32_t started = pros::millis();

    mainPID.doPID(0, P_ERR, [=]() -> float {
      if((pros::millis() - started) > exit) return 0;
      return (fabs(distIn) - dist(start.x, start.y, posTracker.x, posTracker.y));
    }, [=](float output) -> void {
      driveVector(SGN(-distIn) * output, 0);
    });
    driveVector(SGN(-distIn) * output, 0);
  }

  void moveFor(float dist, float exit = 5000) {
    moveFor(dist, { 8, 0, 0.1 }, exit);
  }
}
