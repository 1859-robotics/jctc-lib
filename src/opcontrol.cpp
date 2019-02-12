#include "main.h"
#include "../jctc.hpp"
#include <string>

// only here to not make pros mad and allow for compilation

void opcontrol() {
  std::cout << "heck" << std::endl;
  int i = 0;
  jctc::Logger log(LV_ALIGN_CENTER, "test logger");
  while(true) {
    if(i % 20 == 0) {
      log.log(("hello world " + std::to_string(i)));
    }
    i++;

    pros::delay(20);
  }
}
