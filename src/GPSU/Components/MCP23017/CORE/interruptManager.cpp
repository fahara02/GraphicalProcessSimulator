#include "interruptManager.hpp"
#include "MCPDevice.hpp"
namespace MCP {
InterruptManager::InterruptManager(COMPONENT::MCPDevice *owner)
    : owner_(owner) {
  // Initialization code...
}
} // namespace MCP
