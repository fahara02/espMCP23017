#ifndef MCP_CONSTANTS_HPP
#define MCP_CONSTANTS_HPP
#include "freertos/FreeRTOS.h"
#include "stdint.h"

namespace MCP
{

// MCP DEVICE RELATED
enum class MCP_MODEL
{
	MCP23017,
	MCP23S17,
	MCP23018,
	MCP23S18

};

enum class REG : uint8_t
{
	IODIR = 0x00, //!< I/O Direction Register
	IPOL, //!< Input Polarity Register
	GPINTEN, //!< Interrupt-on-Change Enable Register
	DEFVAL, //!< Default Compare Register
	INTCON, //!< Interrupt Control Register
	IOCON, //!< Configuration Register
	GPPU, //!< Pull-Up Resistor Register
	INTF, //!< Interrupt Flag Register
	INTCAP, //!< Interrupt Captured Value Register
	GPIO, //!< General Purpose I/O Register
	OLAT //!< Output Latch Register
};

enum class PIN
{
	PIN0 = 0,
	PIN1 = 1,
	PIN2 = 2,
	PIN3 = 3,
	PIN4 = 4,
	PIN5 = 5,
	PIN6 = 6,
	PIN7 = 7,
	PIN8 = 8,
	PIN9 = 9,
	PIN10 = 10,
	PIN11 = 11,
	PIN12 = 12,
	PIN13 = 13,
	PIN14 = 14,
	PIN15 = 15,
};
namespace MCP_23X17
{

namespace MCP_23017
{
enum class I2C_CLK : uint32_t
{
	CLK_STD = 100000, // 100KHz
	CLK_HIGH = 400000, // 400 Khz
	CLK_MAX = 17000000 // 1.7 MHZ

};

}; // namespace MCP_23017
namespace MCP_23S17
{
} // namespace MCP_23S17

}; // namespace MCP_23X17

namespace MCP_23X18
{
}; // namespace MCP_23X18

static constexpr uint16_t INT_ERR = 255;
static constexpr uint16_t MCP_ADDRESS_BASE = 0x20;
static constexpr uint16_t DEFAULT_I2C_ADDRESS = 0x20;
static constexpr uint32_t DEFAULT_I2C_CLK_FRQ =
	static_cast<uint32_t>(MCP_23X17::MCP_23017::I2C_CLK::CLK_STD);
static constexpr uint16_t I2C_MASTER_TX_BUF_DISABLE = 0;
static constexpr uint16_t I2C_MASTER_RX_BUF_DISABLE = 0;

static constexpr uint16_t PIN_PER_BANK = 8;
static constexpr uint16_t MAX_PIN = 2 * PIN_PER_BANK;
static constexpr uint16_t MAX_REG_PER_PORT = 11;
static constexpr uint16_t MAX_REG_PER_DEVICE = 2 * MAX_REG_PER_PORT;
static constexpr uint16_t MAX_CALLBACK_PER_REG = 2;
static constexpr uint16_t MAX_EVENT = 100;
static constexpr uint16_t MAX_RETRY = 10;

static constexpr TickType_t DEFAULT_I2C_TIMEOUT = pdMS_TO_TICKS(1000);
static constexpr uint32_t I2C_READ_TIMEOUT_MS = 50;

static constexpr TickType_t MUTEX_TIMEOUT = pdMS_TO_TICKS(50);
static constexpr TickType_t RW_MUTEX_TIMEOUT = pdMS_TO_TICKS(50);
static constexpr TickType_t I2C_MUTEX_TIMEOUT = pdMS_TO_TICKS(50);
static constexpr TickType_t READ_TIMEOUT = pdMS_TO_TICKS(1000);
static constexpr uint16_t MAX_RECOVERY_ATTEMPTS = 50;
enum class MASK
{
	NONE = 0x00,
	ALL = 0xFF,
	EVEN = 0x55, // 01010101
	ODD = 0xAA, // 10101010
	LOWER_HALF = 0x0F, // 00001111
	UPPER_HALF = 0xF0 // 11110000
};

enum class PORT
{
	GPIOA = 0,
	GPIOB = 1,
};

enum class COM_MODE
{
	I2C_MODE,
	SPI_MODE

};

enum class GPIO_MODE
{
	GPIO_OUTPUT = 0,
	GPIO_INPUT = 1,
};
enum class OUTPUT_TYPE
{
	MCP_ACTIVE_PUSHPULL = 0,
	MCP_OPEN_DRAIN = 1
};
enum class INPUT_POLARITY
{
	UNCHANGED = 0,
	INVERTED = 1,
};

enum class PULL_MODE
{
	DISABLE_PULLUP = 0,
	ENABLE_PULLUP = 1
};

enum class INTR_TYPE
{
	INTR_ON_CHANGE = 0,
	INTR_ON_RISING = 1, // SAVE 0 ON DEFVAL and COMPARE
	INTR_ON_FALLING = 2, // SAVE 1 ON DEFVAL and COMPARE
	NONE = -1,
};
enum class INTR_ON_CHANGE_ENABLE
{
	DISABLE_INTR_ON_CHANGE = 0,
	ENABLE_INTR_ON_CHANGE = 1,
	NA = -1,
};
enum class INTR_ON_CHANGE_CONTROL
{
	COMPARE_WITH_OLD_VALUE = 0,
	COMPARE_WITH_DEFVAL = 1,
	NA = -1,
};
enum class INTR_OUTPUT_TYPE
{
	INTR_ACTIVE_LOW = 0, // CONTROLS VIA INTPOL REG when ODR=0
	INTR_ACTIVE_HIGH = 1, // CONTROLS VIA INTPOL REG when ODR=0
	INTR_OPEN_DRAIN = 2, // CONTROLS VIA ODR and CLEAR INTPOL
	NA = -1,
};
enum class DEF_VAL_COMPARE
{
	SAVE_LOGIC_LOW = 0,
	SAVE_LOGIC_HIGH = 1,
	NA = -1,
};

enum class OperationMode
{
	SequentialMode16, // SEQOP = 0, BANK = 0 (16-bit sequential mode)
	SequentialMode8, // SEQOP = 0, BANK = 1( 8-bit sequential mode)
	ByteMode16, // SEQOP = 1, BANK = 0 (Toggles between A/B pairs)
	ByteMode8 // SEQOP = 1, BANK = 1 (8-bit BYTE mode)
};

enum class PairedInterrupt
{
	Disabled,
	Enabled
};
enum class Slew
{
	Enabled,
	Disabled
};
enum class HardwareAddr
{
	Disabled,
	Enabled
}; // Only for SPI
enum class OpenDrain
{
	Disabled,
	Enabled
};
enum class InterruptPolarity
{
	ActiveLow,
	ActiveHigh
};

} // namespace MCP
#endif