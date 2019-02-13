#include "../../include/jctc/logger/logger.hpp"

namespace jctc {
  Logger::Logger(lv_align_t alignment) {
    Logger(alignment, "undefined");
  }

  Logger::Logger(lv_align_t alignment, std::string name) : name(name) {
    label = lv_label_create(lv_scr_act(), NULL);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL);
    lv_obj_align(label, NULL, alignment, 0, 0);
  }

  void Logger::log(const char* text) {
    lv_label_set_text(label, text);
    std::cout << "[" << name << "]: " << text << std::endl;
  }

  void Logger::log(std::string text) {
    lv_label_set_text(label, text.c_str());
  }
}
