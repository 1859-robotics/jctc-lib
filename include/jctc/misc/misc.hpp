#pragma once

#define PI M_PI
#define TAU (PI * 2)

#define SGN(in) (in == 0 ? 0 : (in > 0 ? 1 : -1))
#define TORAD(deg) ((deg) * (PI / 180))
#define TODEG(rad) ((rad) * (180 / PI))
