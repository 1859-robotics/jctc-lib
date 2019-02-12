#pragma once

#include "main.h"
#include <functional>
#include "../misc/misc.hpp"

#ifndef MAX_SPEED
#define MAX_SPEED 127
#endif

#ifndef MIN_SPEED
#define MIN_SPEED 25
#endif

namespace jctc {
  typedef struct {
    float kP;
    float kI;
    float kD;

    float maxSpeed = MAX_SPEED;
    float minSpeed = MIN_SPEED;

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
    bool settled();
    float calculate(float target, float current);
  };
}
