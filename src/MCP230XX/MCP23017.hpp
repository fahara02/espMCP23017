#ifndef MCP23017_HPP
#define MCP23017_HPP
#include "MCPDevice.hpp"
namespace COMPONENT
{
class MCP23017 : public MCP::MCPDevice
{
  public:
	MCP23017(gpio_num_t sda = GPIO_NUM_21, gpio_num_t scl = GPIO_NUM_22,
			 gpio_num_t reset = GPIO_NUM_13) :
		MCP::MCPDevice(MCP::MCP_MODEL::MCP23017, sda, scl, reset)
	{
	}
};

} // namespace COMPONENT

#endif