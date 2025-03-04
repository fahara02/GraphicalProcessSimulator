#ifndef PROCESS_HPP
#define PROCESS_HPP
#include "Services/GraphicManager.hpp"
namespace GPSU {

class ProcessBase {

public:
  ProcessBase() : display_(Service::Display::getInstance()) {}

protected:
  Service::Display &display_;

private:
};

} // namespace GPSU
#endif