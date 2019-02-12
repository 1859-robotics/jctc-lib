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
}
