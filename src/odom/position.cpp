#pragma once
#include "jctc/odom/position.hpp"

namespace jctc {
  namespace odom {
    float dist(Point a, Point b) {
      return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }
    float dot(Point a, Point b) {
      return a.x * b.x + a.y * b.y;
    }
    Point add(Point a, Point b) {
      return { a.x + b.x, a.y + b.y };
    }
    Point sub(Point a, Point b) {
      return { a.x - b.x, a.y - b.y };
    }
    Point mult(Point a, Point b) {
      return { a.x * b.x, a.y * b.y };
    }
    Point mult(Point a, float b) {
      return { a.x * b, a.y * b };
    }
    Point div(Point a, Point b) {
      return { a.x / b.x, a.y / b.y };
    }
    float mag(Point a) {
      return sqrt((a.x * a.x) + (a.y * a.y));
    }
    Point normalize(Point a) {
      if(mag(a) == 0) return a;
      return { a.x / mag(a), a.y / mag(a) };
    }
  }
}
