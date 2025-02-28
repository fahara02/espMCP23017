#ifndef SERIAL_BUS_HPP
#define SERIAL_BUS_HPP
#include "cstdint"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

enum class ERROR_CODE_I2C : uint8_t
{
	SUCCESS = 0,
	DATA_TOO_LONG,
	NACK_ON_ADRESS,
	NACK_ON_DATA,
	OTHERS,
	TIMEOUT
};
class SerialBus
{
  public:
	SerialBus(const SerialBus&) = delete;
	SerialBus& operator=(const SerialBus&) = delete;
	SerialBus(SerialBus&&) = delete;
	SerialBus& operator=(SerialBus&&) = delete;
	virtual void setup(int sda, int scl, uint32_t clock, TickType_t timeout) = 0;

  private:
};
#endif