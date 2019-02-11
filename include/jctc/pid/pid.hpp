#pragma once

#include "main.h"
#include <functional>
#include "jctc/misc/misc.hpp"

#define JCTC_MAX_SPEED 127
#define JCTC_MIN_SPEED 25

namespace jctc {
  typedef struct {
    float kP;
    float kI;
    float kD;

    #ifdef MAX_SPEED
      float maxSpeed = MAX_SPEED;
    #else
      float maxSpeed = JCTC_MAX_SPEED;
    #endif
    #ifdef MAX_SPEED
      float minSpeed = MIN_SPEED;
    #else
      float minSpeed = JCTC_MIN_SPEED;
    #endif

    float epsilonInner = 0.001;
    float epsilonOuter = 0.001;
  } PIDConfig;

  class PID {
  private:

    float lastValue;
  	std::uint32_t lastTime;
    float lastSetPoint;

    int maxSpeed;
    int minSpeed;
  public:
    float kP;
    float kI;
    float kD;

    float epsilonInner;
    float epsilonOuter;

    float sigma;
    PID(PIDConfig config,
        float epsilonInner, float epsilonOuter) :
        kP(config.kP),             kI(config.kI),             kD(config.kD),
        epsilonInner(epsilonInner),             epsilonOuter(epsilonOuter),
        sigma(0),           lastTime(pros::millis()), lastValue(0),
        maxSpeed(config.maxSpeed), minSpeed(config.minSpeed) {};

    void config(float kP, float kI, float kD, float epsilonInner, float epsilonOuter, int maxSpeed, int minSpeed);
    void config(PIDConfig config);
    void doPID(float target, float tolerance, std::function <float()> current, std::function <void(float)> action);
    void debug(std::string name);
    void reset();
    float calculate(float target, float current);
  };
}
