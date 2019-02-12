#pragma once
#include "main.h"

namespace jctc {
  namespace odom {
    struct Point {
      float x, y;
    };

    struct Position {
      Point pos;
      float a;
    };

    // operators
    //TODO: look into operator overloads for some of these
    float dist(Point a, Point b);
    float dot(Point a, Point b);
    Point add(Point a, Point b);
    Point sub(Point a, Point b);
    Point mult(Point a, Point b);
    Point mult(Point a, float b);
    Point div(Point a, Point b);
    float mag(Point a);
    Point normalize(Point a);

    // utils
    void print(Point);
    void print(Position);

    // higher level operations
    Point closest(Point current, Point head, Point target);
    bool withinErr(Point current, Point target, float err);
  }
}
