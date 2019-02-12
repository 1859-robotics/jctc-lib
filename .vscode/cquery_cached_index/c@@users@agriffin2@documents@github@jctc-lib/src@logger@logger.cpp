#include "jctc/logger/logger.hpp"

namespace jctc {
  Logger::Logger() {
    lv_label_set_long_mode(label, LV_LABEL_LONG_BREAK);
    label = lv_obj_create(lv_scr_act(), NULL);
  }

  void Logger::log(const char* text) {
    lv_label_set_text(label, text);
  }
}
