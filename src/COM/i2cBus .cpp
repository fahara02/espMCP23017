#include "COM/i2cBus.hpp"
#define I2C_BUS "I2C_BUS"
namespace MCP
{

SemaphoreHandle_t MCP::I2CBus::i2cMutex = nullptr;
I2CBus::I2CBus(uint8_t addr) : address_(addr), wire_(std::make_unique<TwoWire>(0)) { initMutex(); }

void I2CBus::init()
{

	if(!wire_->begin(sda_, scl_, i2cClock_))
	{
		LOG::ERROR(I2C_BUS, "I2C initialization failed!");
	}
}
void I2CBus::initMutex()
{
	static bool initialized = false;
	if(!initialized)
	{
		i2cMutex = xSemaphoreCreateMutex();
		if(i2cMutex == nullptr)
		{
			LOG::ERROR(I2C_BUS, "Failed to create I2C mutex");
		}
		else
		{
			initialized = true;
		}
	}
}
void I2CBus::Pause() { paused_ = true; }
void I2CBus::Resume() { paused_ = false; }
void I2CBus::reset()
{
	LOG::WARNING(I2C_BUS, "Resetting I2C");
	Pause();
	wire_->clearWriteError();
	wire_->flush();
	gpio_set_level(static_cast<gpio_num_t>(sda_), 1);
	gpio_set_level(static_cast<gpio_num_t>(scl_), 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// Clock SCL to release stuck devices
	for(int i = 0; i < 9; i++)
	{
		gpio_set_level(static_cast<gpio_num_t>(scl_), 0);
		vTaskDelay(1);
		gpio_set_level(static_cast<gpio_num_t>(scl_), 1);
		vTaskDelay(1);
	}

	init();
	Resume();
}
bool I2CBus::i2cBusAvailable() { return wire_->available(); }
void I2CBus::setup(int sda, int scl, uint32_t clock, TickType_t timeout)
{

	sda_ = sda;
	scl_ = scl;
	i2cClock_ = clock;
	timeout_ = timeout;
}

int I2CBus::read_mcp_register(const uint8_t reg, bool map8Bit)
{
	if(paused_)
	{
		return -1;
	}
	SemLock lock(i2cMutex, I2C_MUTEX_TIMEOUT);
	if(!lock.acquired())
	{
		LOG::ERROR(I2C_BUS, "Failed to take mutex");
		return -1;
	}
	uint8_t bytesToRead = 1;
	uint8_t regAddress = reg;

	uint32_t startTime = millis();
	if(!map8Bit)
	{
		// Default 16 bit mapping
		bytesToRead = 2;
		if((reg % 2) != 0)
		{
			regAddress = reg - 1; // Align with Port A
		}
	}

	wire_->beginTransmission(address_);
	wire_->write(regAddress);
	wire_->endTransmission(false);
	wire_->requestFrom((uint8_t)address_, bytesToRead, (uint8_t)true);

	while(wire_->available() < bytesToRead)
	{
		if(millis() - startTime > I2C_READ_TIMEOUT_MS)
		{
			LOG::ERROR(I2C_BUS, "Timeout waiting for data. Expected: %d bytes", bytesToRead);
			return -1; // Signal error
		}
		vTaskDelay(1);
	}

	// as address pointer increases first byte is portA
	uint8_t lowByte = wire_->read(); // PORT A
	uint8_t highByte = (bytesToRead == 2) ? wire_->read() : 0; // PORT B

	return (bytesToRead == 2) ? ((highByte << 8) | (lowByte)) : lowByte;
}
int I2CBus::write_mcp_register(const uint8_t reg, uint16_t value, bool map8Bit)
{
	if(paused_)
	{
		return -1;
	}
	SemLock lock(i2cMutex, I2C_MUTEX_TIMEOUT);
	if(!lock.acquired())
	{
		LOG::ERROR(I2C_BUS, "Failed to take mutex");
		return -1;
	}
	int result = 0;
	uint8_t regAddress = reg; // Default: single register
	uint8_t bytesToWrite = map8Bit ? 1 : 2;

	wire_->beginTransmission(address_);
	// For 16-bit mode, adjust and determine order based on parity.
	if(bytesToWrite == 2)
	{
		if((reg % 2) == 0)
		{
			// Even: assume reg is Port A; low byte -> A, high byte -> B
			regAddress = reg;
			wire_->write(regAddress);
			wire_->write(value & 0xFF); // Port A
			wire_->write((value >> 8) & 0xFF); // Port B
		}
		else
		{
			// Odd: assume reg is Port B; subtract 1 to get base and swap order.
			regAddress = reg - 1;
			wire_->write(regAddress);
			wire_->write((value >> 8) & 0xFF); // Port B now comes first
			wire_->write(value & 0xFF); // Port A second
		}
	}
	else
	{
		// 8-bit mode: simply write one byte.
		wire_->write(regAddress);
		wire_->write(value & 0xFF);
	}

	result = wire_->endTransmission(true);

	if(result != 0)
	{
		LOG::ERROR(I2C_BUS, "I2C transmission failed with error code: %s",
				   Util::ToString::ERROR_I2C(result));
	}
	return result;
}

} // namespace MCP
