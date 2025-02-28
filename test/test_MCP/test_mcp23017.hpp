#ifndef TEST_MCP_23017_HPP
#define TEST_MCP_23017_HPP

#include "MCP23017.hpp"

class MockMCP : public COMPONENT::MCP23017
{
  public:
	MockMCP() : COMPONENT::MCP23017() {}

	static void runAllTests();
	static void initialise();

  private:
	static void testMCPSetting();
	static void testsetInterrupts();
	static void testMCPPinMode();
	static void testMCPDigitalRead();
	static void testMCPDigitalWrite();
};
#endif