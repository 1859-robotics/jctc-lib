#pragma once
// #include <iostream>

namespace jctc {
  struct Vector {
    float x, y;
  };

  struct Position {
    Vector pos;
    float a;
  };

  // operators
  void operator+(Vector, Vector);
  void operator-(Vector, Vector);
  void operator*(Vector, Vector);
  void operator*(Vector, float);
  void operator/(Vector, Vector);
  void operator/(Vector, float);
  void operator<<(std::ostream, Vector); // for printing

  // misc vector operators
  float mag(Vector);
  float dot(Vector);
  float dist(Vector, Vector);
  float dist(Position, Position);
  Vector normalize(Vector);
  float angleBetween(Position, Position);
}
