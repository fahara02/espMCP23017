#ifndef MCP_DEVICE_HPP
#define MCP_DEVICE_HPP

#include "Arduino.h"
#include "CORE/MCP_GPIO_banks.hpp"
#include "CORE/MCP_Primitives.hpp"
#include "CORE/MCP_Registers.hpp"
#include "CORE/RegisterEvents.hpp"
#include "CORE/SemLock.hpp"
#include "Wire.h"

#include "Logger.hpp"
#include "COM/i2cBus.hpp"
#include "CORE/interruptManager.hpp"
#include <array>
#include <memory>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
#include <memory>

#define MCP_TAG "MCPDevice"
namespace std
{
template<>
struct hash<std::tuple<MCP::PORT, MCP::REG>>
{
	std::size_t operator()(const std::tuple<MCP::PORT, MCP::REG>& key) const noexcept
	{
		return std::hash<int>()(static_cast<int>(std::get<0>(key))) ^
			   (std::hash<int>()(static_cast<int>(std::get<1>(key))) << 1);
	}
};
} // namespace std

namespace MCP
{

class MCPDevice : public std::enable_shared_from_this<MCPDevice>
{
  private:
	MCP_MODEL model_;
	Config configuration_;
	Settings settings_;
	Settings defaultSettings_;
	InterruptSetting intrSetting_;

	address_decoder_t decoder_;
	uint8_t address_;

	gpio_num_t sda_;
	gpio_num_t scl_;
	gpio_num_t cs_ = GPIO_NUM_NC;
	gpio_num_t reset_ = GPIO_NUM_33;

	bool bankMode_ = false;
	bool mirrorMode_ = false;
	bool byteMode_ = false;
	bool slewrateDisabled_ = false;
	bool hardwareAddressing_ = false;
	bool opendrainEnabled_ = false;
	bool interruptPolarityHigh_ = false;
	bool errorState_ = false;

	I2CBus& i2cBus_;
	std::shared_ptr<Register> cntrlRegA;
	std::shared_ptr<Register> cntrlRegB;
	std::unique_ptr<GPIO_BANK> gpioBankA;
	std::unique_ptr<GPIO_BANK> gpioBankB;
	std::unique_ptr<InterruptManager> interruptManager_;
	std::unordered_map<std::tuple<PORT, REG>, uint8_t> addressMap_;

	static SemaphoreHandle_t regRWmutex;
	static SemaphoreHandle_t sharedPtrMutex;
	std::atomic<bool> shutdownRequested_{false};
	std::atomic<bool> isActive{true};
	TaskHandle_t eventTaskHandle;
	struct TaskParam
	{
		std::weak_ptr<MCPDevice> weakDevice;
	};

  public:
	MCPDevice(MCP_MODEL model, gpio_num_t sda, gpio_num_t scl, gpio_num_t reset, bool pinA2 = false,
			  bool pinA1 = false, bool pinA0 = false);
	~MCPDevice();
	void init();
	void setupI2c(int sda, int scl, uint32_t clock = DEFAULT_I2C_CLK_FRQ,
				  TickType_t timeout = DEFAULT_I2C_TIMEOUT);
	void updatei2cAddress();
	void configure(const Settings& config);

	bool enableInterrupt();

	void pinMode(const Pin pin, const uint8_t mode);
	void pinMode(const int pin, const uint8_t mode);
	void pinMode(const PORT port, uint8_t pinmask, const uint8_t mode);
	void pinMode(const PORT port, const uint8_t mode);
	template<typename FirstPin, typename... RestPins,
			 typename = std::enable_if_t<(std::is_same_v<FirstPin, MCP::Pin> && ... &&
										  std::is_same_v<RestPins, MCP::Pin>)>>
	void pinMode(const uint8_t mode, FirstPin first, RestPins... rest)
	{

		uint8_t pinmask = generateMask(first, rest...);
		MCP::PORT port = first.getPort();

		pinMode(port, pinmask, mode);
	}

	void digitalWrite(const int pin, const bool level);
	void digitalWrite(const Pin pin, const bool level);
	void digitalWrite(const PORT port, const uint8_t pinmask, const bool level);
	void digitalWrite(const PORT port, const uint8_t level);
	template<typename FirstPin, typename... RestPins,
			 typename = std::enable_if_t<(std::is_same_v<FirstPin, MCP::Pin> && ... &&
										  std::is_same_v<RestPins, MCP::Pin>)>>
	void digitalWrite(const uint8_t level, FirstPin first, RestPins... rest)
	{
		uint8_t pinmask = generateMask(first, rest...);
		PORT port = first.getPort();
		return digitalWrite(port, pinmask, level);
	}

	int digitalRead(const int pin);
	int digitalRead(const Pin pin);
	int digitalRead(const PORT port, const uint8_t pinmask);
	int digitalRead(const PORT port);
	template<typename FirstPin, typename... RestPins,
			 typename = std::enable_if_t<(std::is_same_v<FirstPin, MCP::Pin> && ... &&
										  std::is_same_v<RestPins, MCP::Pin>)>>
	int digitalRead(FirstPin first, RestPins... rest)
	{
		uint8_t pinmask = generateMask(first, rest...);
		PORT port = first.getPort();
		return digitalRead(port, pinmask);
	}

	void invertInput(const int pin, bool invert);
	void invertInput(const Pin pin, bool invert);
	void invertInput(const PORT port, const uint8_t pinmask, bool invert);
	void invertInput(const PORT port, bool invert);
	template<typename FirstPin, typename... RestPins,
			 typename = std::enable_if_t<(std::is_same_v<FirstPin, MCP::Pin> && ... &&
										  std::is_same_v<RestPins, MCP::Pin>)>>
	void invertInput(bool invert, FirstPin first, RestPins... rest)
	{

		uint8_t pinmask = generateMask(first, rest...);
		PORT port = first.getPort();

		return invertInput(port, pinmask, invert);
	}

	void setIntteruptPin(
		PORT port, uint8_t pinmask, uint8_t mcpIntrmode = CHANGE,
		MCP::INTR_OUTPUT_TYPE intrOutMode = MCP::INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH);

	template<typename Pin>
	auto setInterrupts(Pin&& pin) -> std::enable_if_t<std::is_same_v<std::decay_t<Pin>, MCP::Pin>>
	{
		MCP::PORT port = pin.getPort();
		uint8_t mask = (1 << pin.getIndex());
		interruptManager_->updateMask(port, mask);
	}

	template<typename Pin1, typename Pin2, typename... Rest>
	auto setInterrupts(Pin1&& first, Pin2&& second, Rest&&... rest)
		-> std::enable_if_t<std::is_same_v<std::decay_t<Pin1>, MCP::Pin> &&
							std::is_same_v<std::decay_t<Pin2>, MCP::Pin> &&
							(std::is_same_v<std::decay_t<Rest>, MCP::Pin> || ...)>
	{
		PORT port = first.getPort();
		uint8_t pinmask = (1 << first.getIndex());
		interruptManager_->updateMask(port, pinmask);
		port = second.getPort();
		pinmask = (1 << second.getIndex());
		interruptManager_->updateMask(port, pinmask);
	}

	template<typename Pin, typename... Rest>
	auto setInterrupts(Pin&& pin, Rest&&... rest)
		-> std::enable_if_t<std::is_same_v<std::decay_t<Pin>, MCP::Pin> && (sizeof...(Rest) > 0) &&
							!(std::is_invocable_v<Rest, void*> || ...)>
	{
		PORT port = pin.getPort();
		uint8_t mask = (1 << pin.getIndex());
		interruptManager_->updateMask(port, mask);
		setInterrupts(std::forward<Rest>(rest)...);
	}
	template<typename Pin, typename Callback, typename... Rest>
	auto setInterrupts(Pin&& pin, Callback&& callback, Rest&&... rest)
		-> std::enable_if_t<std::is_same_v<std::decay_t<Pin>, MCP::Pin> &&
							std::is_invocable_v<Callback, void*>>
	{
		PORT port = pin.getPort();
		uint8_t localIndex = pin.getIndex();
		uint8_t mask = (1 << localIndex);

		interruptManager_->updateMask(port, mask);
		interruptManager_->registerCallback<void>(port, localIndex, callback, nullptr);

		setInterrupts(std::forward<Rest>(rest)...);
	}

	template<typename Pin, typename Callback, typename T, typename... Rest>
	auto setInterrupts(Pin&& pin, Callback&& callback, T* userData, Rest&&... rest)
		-> std::enable_if_t<std::is_same_v<std::decay_t<Pin>, MCP::Pin> &&
							std::is_invocable_v<Callback, T*>>
	{
		PORT port = pin.getPort();
		uint8_t localIndex = pin.getIndex();
		uint8_t mask = (1 << localIndex);

		interruptManager_->updateMask(port, mask);
		interruptManager_->registerCallback<T>(port, localIndex, callback, userData);

		setInterrupts(std::forward<Rest>(rest)...);
	}

	void setInterrupts(uint8_t mcpIntrmode = CHANGE,
					   MCP::INTR_OUTPUT_TYPE intrOutMode = MCP::INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH)
	{
		updateInterruptSetting(mcpIntrmode, intrOutMode);
		interruptManager_->setup(intrSetting_);
	}

	// Interrupt  handling on esp32 Side
	void attachInterrupt(gpio_num_t pinA, void (*intAHandler)(void*) = nullptr,
						 uint8_t espIntrmode = CHANGE);
	void attachInterrupt(gpio_num_t pinA, void (*intAHandler)(void*) = nullptr,
						 uint8_t espIntrmodeA = CHANGE, gpio_num_t pinB = gpio_num_t::GPIO_NUM_NC,
						 uint8_t espIntrmodeB = CHANGE, void (*intBHandler)(void*) = nullptr);

	template<typename T>
	void attachInterrupt(gpio_num_t pinA, void (*intAHandler)(T*), uint8_t espIntrmodeA,
						 gpio_num_t pinB, uint8_t espIntrmodeB, void (*intBHandler)(T*),
						 T* userDataA, T* userDataB)
	{
		if(pinA != static_cast<gpio_num_t>(-1) && pinB != static_cast<gpio_num_t>(-1))
		{
			intrSetting_.intrSharing = false;
		}
		intrSetting_.modeA_ = coverIntrMode(espIntrmodeA);
		intrSetting_.modeB_ = coverIntrMode(espIntrmodeB);
		interruptManager_->setup(intrSetting_);

		interruptManager_->attachMainHandler<T>(PORT::GPIOA, pinA, intAHandler, userDataA);
		interruptManager_->attachMainHandler<T>(PORT::GPIOB, pinB, intBHandler, userDataB);
	}

	void dumpRegisters() const;
	void resetDevice();
	void resetI2c();
	void reconfigure();
	bool verifyDeviceOperation();

  private:
	void initGPIOPins();
	void initIntrGPIOPins(gpio_int_type_t modeA, gpio_int_type_t modeB);
	void loadSettings();

	MCP::Register* getGPIORegister(REG reg, PORT port);
	MCP::Register* getIntRegister(REG reg, PORT port);

	uint8_t getsavedSettings(PORT port) const;
	uint8_t getRegisterAddress(REG reg, PORT port) const;
	uint8_t getRegisterSavedValue(REG reg, PORT port) const;

	void startEventMonitorTask();
	static void EventMonitorTask(void* param);

	void handleReadEvent(currentEvent* ev);
	void handleWriteEvent(currentEvent* ev);
	void handleSettingChangeEvent(currentEvent* ev);
	void handleBankModeEvent(currentEvent* ev);

	template<typename Pin1, typename Pin2, typename... Rest>
	constexpr uint8_t generateMask(Pin1 first, Pin2 second, Rest... rest)
	{
		static_assert(sizeof...(rest) <= 6, "Too many pins, max is 8");
		PORT port = first.getPort();
		assert(((second.getPort() == port)));
		assert((... && (rest.getPort() == port)));

		return (1 << first.getIndex()) | (1 << second.getIndex()) |
			   (0 | ... | (1 << rest.getIndex()));
	}

	std::unordered_map<std::tuple<PORT, REG>, uint8_t> populateAddressMap(bool bankMode);

	void updateAddressMap(bool bankMode);

	bool resetInterruptRegisters();
	static void IRAM_ATTR defaultIntAHandler(void* arg);
	static void IRAM_ATTR defaultIntBHandler(void* arg);

	void setupDefaultIntterupt(INTR_TYPE type = INTR_TYPE::INTR_ON_CHANGE,
							   INTR_OUTPUT_TYPE outtype = INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH,
							   PairedInterrupt sharedIntr = PairedInterrupt::Disabled);

	uint8_t getInterruptMask(PORT port) { return interruptManager_->getMask(port); }
	void updateInterruptSetting(uint8_t mcpIntrmode, INTR_OUTPUT_TYPE intrOutMode);
	gpio_int_type_t coverIntrMode(uint8_t mode);

	std::pair<MCP::PORT, uint8_t> getPortAndMask(int pin);
};

} // namespace MCP

#endif
