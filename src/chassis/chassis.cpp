#include "jctc/chassis/chassis.hpp"

namespace jctc {
  void Chassis::driveTo(odom::Point target, float rotScalar, int timeout, float err) {
    const odom::Position state = tracker.getPos();
    float tA = rollPI(atan2(target.y - state.pos.y, target.x - state.pos.x) - state.a);

  }
}
