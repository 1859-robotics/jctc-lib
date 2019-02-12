#pragma once
#include "main.h"
#include <string>

namespace jctc {
  class Logger {
  private:
    lv_obj_t * label;
  public:
    Logger();
    void log(const char* text);
  };
}
