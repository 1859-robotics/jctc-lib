#pragma once
#include "main.h"
#include "jctc/odom/position.hpp"

namespace jctc {
  namespace odom {
    class Tracker {
    private:
      pros::ADIEncoder *lEncoder;
      pros::ADIEncoder *rEncoder;
      pros::ADIEncoder *mEncoder;

      int prevLEncoderVal;
      int prevREncoderVal;
      int prevMEncoderVal;

      float sL;
      float sR;
      float sM;
      float encoderScalar;

      Position state;

    public:
      Tracker(pros::ADIEncoder *lEncoder, pros::ADIEncoder *rEncoder, pros::ADIEncoder *mEncoder,
              float sL, float sR, float sM, float encoderScalar) :
        lEncoder(lEncoder), rEncoder(rEncoder), mEncoder(mEncoder),
        sL(sL), sR(sR), sM(sM), encoderScalar(encoderScalar) {};

      void debug();
      void step();
      void setPos(float x, float y, float a);
      void setPos(Point pt, float a);
      void setPos(Position pos);
      void reset();

    };
  }
}
