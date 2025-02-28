#ifndef MCP_PRIMITIVES_HPP
#define MCP_PRIMITIVES_HPP
#include "MCP_Constants.hpp"
#include "Util/Utility.hpp"
#include "atomic"
#include <array>
#include <cassert>

namespace MCP
{

//
struct Pin
{

	const PIN pinEnum;
	const PORT port;
	const uint8_t pinNumber;
	const uint8_t mask;
	// Constructor

	constexpr Pin(PIN pin) :
		pinEnum(pin), port(Util::getPortFromPin(pin)), pinNumber(Util::getPinIndex(pin)),
		mask(1 << (Util::getPinIndex(pin))), state(false), interruptEnabled(false)
	{
	}

	constexpr Pin() : Pin(static_cast<PIN>(0)) {}

	// Full copy constructor
	constexpr Pin(const Pin& other) :
		pinEnum(other.pinEnum), port(other.port), pinNumber(other.pinNumber), mask(other.mask),
		state(other.state), // Copy atomic state
		interruptEnabled(false)
	{
	}

	constexpr Pin& operator=(const Pin& other)
	{
		if(this != &other)
		{

			interruptEnabled = other.interruptEnabled;
			state = other.state;
		}
		return *this;
	}

	// Core constants getters
	constexpr PIN getEnum() const { return pinEnum; }
	constexpr uint8_t getPinNumber() const { return pinNumber; }
	constexpr uint8_t getMask() const { return mask; }
	constexpr PORT getPort() const { return port; }
	constexpr uint8_t getIndex() const { return Util::getPinIndex(pinEnum); }

	// State management
	bool getState() const { return state; }
	void setState(bool newState) { state = newState; }

  private:
	bool state;
	bool interruptEnabled;
};

//

} // namespace MCP
// GPA Pins (PIN0 to PIN7)
constexpr MCP::Pin GPA0(MCP::PIN::PIN0);
constexpr MCP::Pin GPA1(MCP::PIN::PIN1);
constexpr MCP::Pin GPA2(MCP::PIN::PIN2);
constexpr MCP::Pin GPA3(MCP::PIN::PIN3);
constexpr MCP::Pin GPA4(MCP::PIN::PIN4);
constexpr MCP::Pin GPA5(MCP::PIN::PIN5);
constexpr MCP::Pin GPA6(MCP::PIN::PIN6);
constexpr MCP::Pin GPA7(MCP::PIN::PIN7);

// GPB Pins (PIN8 to PIN15)
constexpr MCP::Pin GPB0(MCP::PIN::PIN8);
constexpr MCP::Pin GPB1(MCP::PIN::PIN9);
constexpr MCP::Pin GPB2(MCP::PIN::PIN10);
constexpr MCP::Pin GPB3(MCP::PIN::PIN11);
constexpr MCP::Pin GPB4(MCP::PIN::PIN12);
constexpr MCP::Pin GPB5(MCP::PIN::PIN13);
constexpr MCP::Pin GPB6(MCP::PIN::PIN14);
constexpr MCP::Pin GPB7(MCP::PIN::PIN15);
#endif