#include "../../include/jctc/misc/misc.hpp"

namespace jctc {
  float rollPI(float angle) {
    return angle - TAU * std::floor((angle + PI) * (1.0 / TAU));
  }
}
