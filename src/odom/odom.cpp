#pragma once
#include "jctc/odom/odom.hpp"

namespace jctc {
  namespace odom {
    void Tracker::debug() {
      print(state);
    }

    void Tracker::step() {
      float newL = lEncoder->get_value();
      float newR = rEncoder->get_value();
      float newM = mEncoder->get_value();

      float dL = (prevLEncoderVal - newL) / encoderScalar;
      float dR = (prevREncoderVal - newR) / encoderScalar;
      float dM = (prevMEncoderVal - newM) / encoderScalar;

      prevLEncoderVal = newL;
      prevREncoderVal = newR;
      prevMEncoderVal = newM;

      float dA = ((dR - dL) / (sL + sR)); // TODO: reset nodes?

      float avgA = state.a + (dA / 2);

      float dS = (dL + dR) / 2;

      float localOffX, localOffY;

      if(dA != 0) {
        localOffX = 2 * sin(dA / 2) * ((dS / dA) + sR);
        localOffY = 2 * sin(dA / 2) * ((dM / dA) + sM);
      } else {
        localOffX = dS;
        localOffY = dM;
      }

      float polarR = sqrt((localOffX * localOffX) + (localOffY * localOffY));
      float polarA = atan2(localOffY, localOffX) - avgA;

      float dX = cos(polarA) * polarR;
      float dY = sin(polarA) * polarR;

      this->state.pos.x += dX;
      this->state.pos.y += dY;
      this->state.a += dA;
    }

    void Tracker::setPos(float x, float y, float a) {
      state.pos.x = x;
      state.pos.y = y;
      state.a = a;
    }
    void Tracker::setPos(Point pt, float a) {
      state.pos = pt;
      state.a = a;
    }
    void Tracker::setPos(Position pos) {
      state = pos;
    }

    void Tracker::reset() {
      lEncoder->reset();
      rEncoder->reset();
      mEncoder->reset();
      setPos({ 0, 0, 0 });
    }

    Position Tracker::getPos() {
      return state;
    }
  }
}
