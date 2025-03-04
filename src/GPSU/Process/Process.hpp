#ifndef PROCESS_HPP
#define PROCESS_HPP
#include "GUI/GUI.hpp"
namespace GPSU {

class ProcessBase {

public:
  ProcessBase() : display_(GUI::Display::getInstance()) {}

protected:
  GUI::Display &display_;

private:
};

} // namespace GPSU
#endif