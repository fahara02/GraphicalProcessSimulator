#ifndef MCP23017_HPP
#define MCP23017_HPP
#include "MCPDevice.hpp"
namespace COMPONENT {
class MCP23017 : public MCP::MCPDevice {
public:
  MCP23017() : MCP::MCPDevice(MCP::MCP_MODEL::MCP23017) {}
};

} // namespace COMPONENT

#endif