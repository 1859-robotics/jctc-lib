#pragma once
#include "main.h"
#include <string>

namespace jctc {
  class Logger {
  private:
    lv_obj_t *label;
    std::string name;
  public:
    Logger(lv_align_t alignment);
    Logger(lv_align_t alignment, std::string name);
    void log(const char* text);
    void log(std::string text);
  };
}
