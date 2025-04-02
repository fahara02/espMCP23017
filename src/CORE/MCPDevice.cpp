#include "CORE/MCPDevice.hpp"
#include "climits"
#include "driver/gpio.h"
#include "Logger.hpp"
#include "esp_task_wdt.h"
namespace MCP
{
static int GLOBAL_ERROR_COUNT = 0;
SemaphoreHandle_t MCPDevice::regRWmutex = xSemaphoreCreateMutex();
MCPDevice::MCPDevice(MCP_MODEL model, gpio_num_t sda, gpio_num_t scl, gpio_num_t reset, bool pinA2,
					 bool pinA1, bool pinA0) :
	model_(model), sda_(sda), scl_(scl), reset_(reset), configuration_(model),
	settings_(configuration_.getSettings()), defaultSettings_(Settings(model)),
	decoder_(model, pinA2, pinA1, pinA0), address_(decoder_.getDeviceAddress(false)),
	i2cBus_(I2CBus::getInstance(address_)),
	cntrlRegA(std::make_shared<Register>(model_, REG::IOCON, PORT::GPIOA, bankMode_)),
	cntrlRegB(std::make_shared<Register>(model_, REG::IOCON, PORT::GPIOB, bankMode_)),
	gpioBankA(std::make_unique<GPIO_BANK>(PORT::GPIOA, model, cntrlRegA)),
	gpioBankB(std::make_unique<GPIO_BANK>(PORT::GPIOB, model, cntrlRegB)),

	interruptManager_(std::make_unique<InterruptManager>(model, i2cBus_, cntrlRegA, cntrlRegB)),
	addressMap_(populateAddressMap(bankMode_))

{
}
MCPDevice::~MCPDevice() = default;
void MCPDevice::configure(const Settings& setting)
{
	if(cntrlRegA && cntrlRegB)
	{
		if(configuration_.configure(setting))
		{
			settings_ = configuration_.getSettings();
			loadSettings();
		}
	}
}
void MCPDevice::reconfigure() { configure(settings_); }

void MCPDevice::updatei2cAddress()
{
	bool haen = configuration_.getBitField(Config::Field::HAEN);
	address_ = decoder_.getDeviceAddress(haen);
}

void MCPDevice::loadSettings()
{

	if(settings_ == defaultSettings_)
	{
		return;
	}
	uint8_t result = 0;

	// Determine bankMode_ and byteMode_ at once
	switch(settings_.opMode)
	{
		case MCP::OperationMode::SequentialMode16: // SEQOP = 0, BANK = 0
			bankMode_ = false;
			byteMode_ = false;
			break;

		case MCP::OperationMode::SequentialMode8: // SEQOP = 0, BANK = 1
			bankMode_ = true;
			byteMode_ = false;
			break;

		case MCP::OperationMode::ByteMode16: // SEQOP = 1, BANK = 0
			bankMode_ = false;
			byteMode_ = true;
			break;

		case MCP::OperationMode::ByteMode8: // SEQOP = 1, BANK = 1
			bankMode_ = true;
			byteMode_ = true;
			break;
	}
	uint8_t cntrlAddressA = cntrlRegA->getAddress();
	uint8_t cntrlAddressB = cntrlRegB->getAddress();
	// Step 1: If switching to banked mode, write only BANK bit first
	if(bankMode_)
	{
		result |= i2cBus_.write_mcp_register(cntrlAddressA, static_cast<uint8_t>(0x80), bankMode_);
		updateAddressMap(bankMode_);
		gpioBankA->updateBankMode(bankMode_);
		gpioBankB->updateBankMode(bankMode_);
		interruptManager_->updateBankMode(bankMode_);
	}

	// Step 2: Update remaining settings using configuration struct value
	uint8_t updatedSetting = configuration_.getSettingValue();

	cntrlRegA->configure<MCP::REG::IOCON>(updatedSetting);
	cntrlRegB->configure<MCP::REG::IOCON>(updatedSetting);

	if(bankMode_)
	{
		// In BANK mode (8-bit register mapping), write separately to each bank
		result |= i2cBus_.write_mcp_register(cntrlAddressA, updatedSetting, bankMode_);
		result |= i2cBus_.write_mcp_register(cntrlAddressB, updatedSetting, bankMode_);
	}
	else
	{
		// In 16-bit register mapping, write to both registers in sequence
		result |= i2cBus_.write_mcp_register(cntrlAddressA, updatedSetting, bankMode_);
		result |= i2cBus_.write_mcp_register(cntrlAddressA + 1, updatedSetting, bankMode_);
	}

	// Step 4: Update other MCP settings
	mirrorMode_ = (settings_.mirror == PairedInterrupt::Enabled);
	slewrateDisabled_ = (settings_.slew == Slew::Disabled);
	hardwareAddressing_ = (settings_.haen == HardwareAddr::Enabled);
	if(hardwareAddressing_)
	{
		updatei2cAddress();
	}
	opendrainEnabled_ = (settings_.odr == OpenDrain::Enabled);
	interruptPolarityHigh_ = (settings_.intpol == MCP::InterruptPolarity::ActiveHigh);

	// Step 5: Handle result logging
	if(result != 0)
	{
		LOG::ERROR(MCP_TAG, "New settings change failed, reverting to defaults");
		configuration_.configureDefault();
	}
	else
	{
		LOG::ERROR(MCP_TAG, "Successfully changed the settings");
	}
}

void MCPDevice::resetDevice()
{
	LOG::WARNING(MCP_TAG, "resetting the device");
	gpio_set_level(reset_, 1);
	vTaskDelay(10);
	gpio_set_level(reset_, 0);
	vTaskDelay(100);
}
void MCPDevice::resetI2c()
{
	LOG::WARNING(MCP_TAG, "resetting I2c");
	i2cBus_.reset();
}
void MCPDevice::init()
{
	initGPIOPins();
	int sda = static_cast<int>(sda_);
	int scl = static_cast<int>(scl_);
	setupI2c(sda, scl, DEFAULT_I2C_CLK_FRQ, DEFAULT_I2C_TIMEOUT);
	i2cBus_.init();
	EventManager::initializeEventGroups();
	startEventMonitorTask(this);
	interruptManager_->setupInterruptMask(PORT::GPIOB, 0XFF);
	setupDefaultIntterupt();
	resetInterruptRegisters();
}

void MCPDevice::setupI2c(int sda, int scl, uint32_t clock, TickType_t timeout)
{
	if(sda != -1 && scl != -1)
	{
		LOG::INFO(MCP_TAG, "Setting up I2c with PIN %d as SDA and %d as SCl ", sda, scl);
		i2cBus_.setup(sda, scl, clock, timeout);
	}
	else
	{
		LOG::ERROR(MCP_TAG, "Invalid settings for I2c");
	}
}

void MCPDevice::initGPIOPins()
{
	if(reset_ != gpio_num_t::GPIO_NUM_NC)
	{
		gpio_config_t io_conf = {};
		io_conf.pin_bit_mask = (1ULL << reset_);
		io_conf.mode = GPIO_MODE_OUTPUT;
		io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
		io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
		io_conf.intr_type = GPIO_INTR_DISABLE;

		gpio_config(&io_conf);
		gpio_set_level(reset_, 0);
	}

	// Initialize SDA and SCL if they are not NC
	if(sda_ != gpio_num_t::GPIO_NUM_NC && scl_ != gpio_num_t::GPIO_NUM_NC)
	{
		gpio_config_t io_conf = {};
		io_conf.pin_bit_mask = (1ULL << sda_) | (1ULL << scl_);
		io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD; // Open-drain for I2C
		io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // I2C needs pull-ups
		io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
		io_conf.intr_type = GPIO_INTR_DISABLE;

		gpio_config(&io_conf);
	}
}

bool MCPDevice::enableInterrupt() { return interruptManager_->enableInterrupt(); }
bool MCPDevice::resetInterruptRegisters() { return interruptManager_->resetInterruptRegisters(); }
void MCPDevice::startEventMonitorTask(MCPDevice* device)
{
	if(!device)
	{
		ESP_LOGE(MCP_TAG, "no_device");
	}
	else
	{
		xTaskCreatePinnedToCore(EventMonitorTask, "EventMonitorTask", 8192, device, 5,
								&eventTaskHandle, 0);
	}
}

void MCPDevice::pinMode(const PORT port, const uint8_t pinmask, const uint8_t mode)
{
	GPIO_BANK* gpioBank = (port == PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
	auto* cntrlReg = (port == PORT::GPIOA) ? cntrlRegA.get() : cntrlRegB.get();
	if(!gpioBank)
	{
		assert(false && "Invalid port");
		return;
	}
	gpioBank->setGeneralMask(pinmask);
	switch(mode)
	{
		case INPUT:
			gpioBank->setPinDirection(pinmask, GPIO_MODE::GPIO_INPUT);
			break;
		case INPUT_PULLUP:
			gpioBank->setPinDirection(pinmask, GPIO_MODE::GPIO_INPUT);
			gpioBank->setPullup(pinmask, PULL_MODE::ENABLE_PULLUP);
			break;
		case INPUT_PULLDOWN:

			ESP_LOGE(MCP_TAG, "PullDown not available in MCP devices, defaulting to INPUT.");
			break;
		case OUTPUT:
			gpioBank->setPinDirection(pinmask, GPIO_MODE::GPIO_OUTPUT);
			break;
		case OUTPUT_OPEN_DRAIN:
			gpioBank->setPinDirection(pinmask, GPIO_MODE::GPIO_OUTPUT);
			cntrlReg->setOpenDrain<REG::IOCON>(true);
			break;
		default:
			assert(false && "Invalid mode");
			break;
	}
}
void MCPDevice::pinMode(const PORT port, const uint8_t mode)
{
	GPIO_BANK* gpioBank = (port == PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
	return pinMode(port, gpioBank->getGeneralMask(), mode);
}

void MCPDevice::pinMode(const Pin pin, const uint8_t mode)
{
	pinMode(Util::getPortFromPin(pin.getEnum()), pin.getMask(), mode);
}

void MCPDevice::pinMode(const int pin, const uint8_t mode)
{
	auto [port, mask] = getPortAndMask(pin);
	pinMode(port, mask, mode);
}

void MCPDevice::digitalWrite(const PORT port, const uint8_t pinmask, const uint8_t level)
{
	GPIO_BANK* gpioBank = (port == PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
	gpioBank->setGeneralMask(pinmask);
	gpioBank->setOutput(level);
}
void MCPDevice::digitalWrite(const PORT port, const uint8_t level)
{
	GPIO_BANK* gpioBank = (port == PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
	gpioBank->setOutput(level);
}
void MCPDevice::digitalWrite(const Pin pin, const bool level)
{

	MCP::PORT port = Util::getPortFromPin(pin.getEnum());
	GPIO_BANK* gpioBank = (port == PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
	gpioBank->setPinState(pin.getMask(), level);
}

void MCPDevice::digitalWrite(const int pin, const bool level)
{
	auto [port, mask] = getPortAndMask(pin);
	GPIO_BANK* gpioBank = (port == PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
	gpioBank->setPinState(mask, level);
}

uint8_t MCPDevice::digitalRead(const PORT port, const uint8_t pinmask)
{
	GPIO_BANK* gpioBank = (port == PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
	gpioBank->setGeneralMask(pinmask);
	return gpioBank->getPinState(pinmask);
}
bool MCPDevice::digitalRead(const Pin pin)
{
	return digitalRead(Util::getPortFromPin(pin.getEnum()), pin.getMask());
}
uint8_t MCPDevice::digitalRead(const PORT port)
{
	GPIO_BANK* gpioBank = (port == PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
	return gpioBank->getPinState();
}

bool MCPDevice::digitalRead(const int pin)
{
	auto [port, mask] = getPortAndMask(pin);
	return digitalRead(port, mask);
}

void MCPDevice::invertInput(const PORT port, const uint8_t pinmask, bool invert)
{
	GPIO_BANK* gpioBank = (port == PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();
	gpioBank->setGeneralMask(pinmask);
	gpioBank->setInputPolarity(pinmask,
							   invert ? INPUT_POLARITY::INVERTED : INPUT_POLARITY::UNCHANGED);
}
void MCPDevice::invertInput(const PORT port, bool invert)
{
	GPIO_BANK* gpioBank = (port == PORT::GPIOA) ? gpioBankA.get() : gpioBankB.get();

	gpioBank->setInputPolarity(invert ? INPUT_POLARITY::INVERTED : INPUT_POLARITY::UNCHANGED);
}

void MCPDevice::invertInput(const Pin pin, bool invert)
{
	invertInput(Util::getPortFromPin(pin.getEnum()), pin.getMask(), invert);
}

void MCPDevice::invertInput(int pin, bool invert)
{
	auto [port, mask] = getPortAndMask(pin);
	invertInput(port, mask, invert);
}

void MCPDevice::EventMonitorTask(void* param)
{
	MCPDevice* device = static_cast<MCPDevice*>(param);
	uint8_t recovery_attempts = 0;
	TickType_t recovery_delay = pdMS_TO_TICKS(500);

	while(true)
	{
		if(device->errorState_)
		{
			GLOBAL_ERROR_COUNT++;
			LOG::WARNING(MCP_TAG, "Device in error state %d, attempt %d, waiting for recovery...",
						 GLOBAL_ERROR_COUNT, recovery_attempts + 1);
			esp_task_wdt_reset();

			device->resetDevice();
			device->resetI2c();
			device->reconfigure();

			bool recovery_success = device->verifyDeviceOperation();

			if(recovery_success)
			{
				LOG::SUCCESS(MCP_TAG, "Device successfully recovered after %d attempts",
							 recovery_attempts + 1);
				device->errorState_ = false;
				recovery_attempts = 0;
				vTaskDelay(pdMS_TO_TICKS(100)); // Short delay after successful recovery
			}
			else
			{
				recovery_attempts++;

				if(recovery_attempts >= MAX_RECOVERY_ATTEMPTS)
				{
					LOG::ERROR(MCP_TAG,
							   "Maximum recovery attempts reached. Performing system reset");
					// Consider more drastic measures here
					// For example, you might want to reboot the entire ESP32
					// esp_restart();  // Uncomment if you want to reboot the ESP32

					// Or just reset the recovery counter and continue trying
					recovery_attempts = 0;
					recovery_delay = pdMS_TO_TICKS(2000); // Longer delay after max attempts
				}
				else
				{
					// Exponential backoff
					recovery_delay = pdMS_TO_TICKS(500 * (1 << recovery_attempts));
				}

				vTaskDelay(recovery_delay);
				continue;
			}
		}
		// Wait for events.
		const EventBits_t CHECK_BITS_MASK =
			static_cast<EventBits_t>(RegisterEvent::READ_REQUEST) |
			static_cast<EventBits_t>(RegisterEvent::WRITE_REQUEST) |
			static_cast<EventBits_t>(RegisterEvent::BANK_MODE_CHANGED) |
			static_cast<EventBits_t>(RegisterEvent::SETTINGS_CHANGED) |
			static_cast<EventBits_t>(RegisterEvent::ERROR);
		EventBits_t eventBits =
			xEventGroupWaitBits(EventManager::registerEventGroup, CHECK_BITS_MASK,
								pdFALSE, // Don't clear bits automatically
								pdFALSE, // Wait for any event
								MUTEX_TIMEOUT);
		// Handle ERROR event
		if(eventBits & static_cast<EventBits_t>(RegisterEvent::ERROR))
		{
			if(device->errorState_)
			{
				EventManager::clearAllErrorEvents();
				vTaskDelay(pdMS_TO_TICKS(500));
			}
			currentEvent* event = EventManager::getEvent(RegisterEvent::ERROR);
			if(event)
			{
				LOG::INFO(MCP_TAG, "EventMonitorTask received eventBits: 0x%08X", eventBits);
				LOG::ERROR(MCP_TAG, "ERROR event detected! Value: 0x%02X", event->data);

				// Potential recovery action:
				// - Reset I2C bus
				// - Reinitialize MCP23017 settings

				EventManager::acknowledgeEvent(event);
				EventManager::clearBits(RegisterEvent::ERROR);
				device->errorState_ = true;
				vTaskDelay(pdMS_TO_TICKS(500));
			}
		}

		// Reset watchdog frequently

		// Handle READ_REQUEST.
		if(eventBits & static_cast<EventBits_t>(RegisterEvent::READ_REQUEST))
		{
			SemLock lock(regRWmutex, MUTEX_TIMEOUT);
			if(!lock.acquired())
			{
				LOG::ERROR(MCP_TAG, "Failed to acquire mutex in READ_REQUEST");
				continue;
			}
			currentEvent* event = EventManager::getEvent(RegisterEvent::READ_REQUEST);
			if(event && event->event != RegisterEvent::ERROR)
			{
				device->handleReadEvent(event);
			}
		}

		// Handle WRITE_REQUEST.
		if(eventBits & static_cast<EventBits_t>(RegisterEvent::WRITE_REQUEST))
		{
			SemLock lock(regRWmutex, MUTEX_TIMEOUT);
			if(!lock.acquired())
			{
				LOG::ERROR(MCP_TAG, "Failed to acquire mutex in WRITE_REQUEST");
				continue;
			}
			currentEvent* event = EventManager::getEvent(RegisterEvent::WRITE_REQUEST);
			if(event && event->event != RegisterEvent::ERROR)
			{
				device->handleWriteEvent(event);
			}
		}

		// Handle BANK_MODE_CHANGED.
		if(eventBits & static_cast<EventBits_t>(RegisterEvent::BANK_MODE_CHANGED))
		{
			SemLock lock(regRWmutex, MUTEX_TIMEOUT);
			if(!lock.acquired())
			{
				LOG::ERROR(MCP_TAG, "Failed to acquire mutex in BANK_MODE_CHANGED");
				continue;
			}
			currentEvent* event = EventManager::getEvent(RegisterEvent::BANK_MODE_CHANGED);
			if(event && event->event != RegisterEvent::ERROR)
			{
				device->handleBankModeEvent(event);
			}
		}

		// Handle SETTINGS_CHANGED.
		if(eventBits & static_cast<EventBits_t>(RegisterEvent::SETTINGS_CHANGED))
		{
			SemLock lock(regRWmutex, MUTEX_TIMEOUT);
			if(!lock.acquired())
			{
				LOG::ERROR(MCP_TAG, "Failed to acquire mutex in SETTINGS_CHANGED");
				continue;
			}
			currentEvent* event = EventManager::getEvent(RegisterEvent::SETTINGS_CHANGED);
			if(event && event->event != RegisterEvent::ERROR)
			{
				device->handleSettingChangeEvent(event);
			}
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
	vTaskDelete(NULL);
}
void MCPDevice::handleBankModeEvent(currentEvent* ev)
{
	uint8_t regAddress = ev->regIdentity.regAddress;
	uint8_t settings = ev->data;
	int value = i2cBus_.write_mcp_register(regAddress, settings, bankMode_);
	if(value == -1)
	{
		LOG::ERROR(MCP_TAG, "Write failed for id=%d ; Invalid data received\n", ev->id);

		return;
	}

	Serial.printf("New Event %d BankMode changed \n", ev->id);
	EventManager::acknowledgeEvent(ev);
	EventManager::clearBits(RegisterEvent::BANK_MODE_CHANGED);
}

void MCPDevice::handleReadEvent(currentEvent* ev)
{
	REG reg = ev->regIdentity.reg;
	uint8_t currentAddress = ev->regIdentity.regAddress;
	PORT port = ev->regIdentity.port;
	bool intrFunctions = ev->intteruptFunction_;
	uint8_t regAddress = 0;

	int value = i2cBus_.read_mcp_register(currentAddress, bankMode_);
	if(value == -1)
	{
		LOG::ERROR(MCP_TAG, "Read failed for id=%d ; Invalid data received\n", ev->id);
		EventManager::createEvent(ev->regIdentity, RegisterEvent::DATA_RECEIVED, 0xFF);
		EventManager::clearBits(RegisterEvent::READ_REQUEST);

		return;
	}

	if(!bankMode_)
	{
		if((currentAddress % 2) != 0)
		{
			regAddress = currentAddress - 1; // Align with Port A
		}
		else
		{
			regAddress = currentAddress;
		}

		uint8_t valueA = static_cast<uint16_t>(value) & 0xFF; // extract lowByte
		uint8_t valueB = (static_cast<uint16_t>(value) >> 8) & 0xFF; // extract highbyte

		LOG::SUCCESS(MCP_TAG,
					 "Read success for REG %s id=%d ; address %02X with value %02X "
					 "PORTA =%02X "
					 "and PORTB =%02X  \n",
					 Util::ToString::REG(reg), ev->id, currentAddress, value, valueA, valueB);

		if(intrFunctions)
		{
			uint8_t newValue = port == PORT::GPIOA ? valueA : valueB;

			interruptManager_->updateRegisterValue(port, currentAddress, newValue);
		}
		else
		{
			gpioBankA->updateRegisterValue(regAddress, valueA);
			gpioBankB->updateRegisterValue(regAddress + 1, valueB);
		}
	}
	else
	{
		if(intrFunctions)
		{
			interruptManager_->updateRegisterValue(port, currentAddress,
												   static_cast<uint8_t>(value));
		}
		else
		{
			if(port == PORT::GPIOA)
			{
				gpioBankA->updateRegisterValue(currentAddress, static_cast<uint8_t>(value));
			}
			else
			{
				gpioBankB->updateRegisterValue(currentAddress, static_cast<uint8_t>(value));
			}
		}
	}

	EventManager::acknowledgeEvent(ev);
	EventManager::clearBits(RegisterEvent::READ_REQUEST);

	EventManager::createEvent(ev->regIdentity, RegisterEvent::DATA_RECEIVED, value);
	errorState_ = false;
}

void MCPDevice::handleWriteEvent(currentEvent* ev)
{
	uint8_t reg = ev->regIdentity.regAddress;
	uint16_t value = ev->data; // Use 16-bit storage
	uint8_t result = i2cBus_.write_mcp_register(reg, value, bankMode_);

	if(result == 0)
	{

		LOG::SUCCESS(MCP_TAG, "New Write success for address 0x%02X for id=%d ; \n", reg, ev->id);
		EventManager::acknowledgeEvent(ev);
		errorState_ = false;
	}
	else
	{
		LOG::ERROR(MCP_TAG, "New Write failed for id=%d ; \n", ev->id);
		EventManager::acknowledgeEvent(ev);
		EventManager::createEvent(ev->regIdentity, RegisterEvent::ERROR);
	}

	EventManager::clearBits(RegisterEvent::WRITE_REQUEST);
}

void MCPDevice::handleSettingChangeEvent(currentEvent* ev)
{

	uint8_t reg = ev->regIdentity.regAddress;
	uint16_t settings = ev->data;
	uint8_t result = i2cBus_.write_mcp_register(reg, settings, bankMode_);
	if(result == 0)
	{
		LOG::SUCCESS(REG_TAG, "New Setting Event sucessfull id=%d ; \n", ev->id);
		EventManager::acknowledgeEvent(ev);
	}
	else
	{
		Serial.printf("New Setting Event failed for id=%d ; \n", ev->id);
	}

	EventManager::clearBits(RegisterEvent::SETTINGS_CHANGED);
}

Register* MCPDevice::getGPIORegister(REG reg, PORT port)
{
	if(port == PORT::GPIOA)
	{

		return gpioBankA->getRegisterForUpdate(reg);
	}
	else
	{

		return gpioBankB->getRegisterForUpdate(reg);
	}
}
Register* MCPDevice::getIntRegister(REG reg, PORT port)
{

	return interruptManager_->getRegister(port, reg);
}
uint8_t MCPDevice::getsavedSettings(PORT port) const
{
	return port == MCP::PORT::GPIOA ? cntrlRegA->getSavedValue() : cntrlRegB->getSavedValue();
}

uint8_t MCPDevice::getRegisterAddress(REG reg, PORT port) const
{
	auto key = std::make_tuple(port, reg);
	auto it = addressMap_.find(key);

	if(it != addressMap_.end())
	{
		return it->second;
	}
	else
	{

		return 0xFF;
	}
}

uint8_t MCPDevice::getRegisterSavedValue(REG reg, PORT port) const
{
	if(port == MCP::PORT::GPIOA)
	{
		return gpioBankA->getSavedValue(reg);
	}
	else
	{
		return gpioBankB->getSavedValue(reg);
	}
}

std::unordered_map<std::tuple<PORT, REG>, uint8_t> MCPDevice::populateAddressMap(bool bankMode)
{
	std::unordered_map<std::tuple<PORT, REG>, uint8_t> addressMap;

	const std::vector<REG> registers = {REG::IODIR,	 REG::IPOL,	 REG::GPINTEN, REG::DEFVAL,
										REG::INTCON, REG::IOCON, REG::GPPU,	   REG::INTF,
										REG::INTCAP, REG::GPIO,	 REG::OLAT};

	for(const auto reg: registers)
	{
		addressMap[{PORT::GPIOA, reg}] = Util::calculateAddress(reg, PORT::GPIOA, bankMode);
		addressMap[{PORT::GPIOB, reg}] = Util::calculateAddress(reg, PORT::GPIOB, bankMode);
	}

	return addressMap;
}

void MCPDevice::updateAddressMap(bool bankMode)
{
	addressMap_.clear();
	addressMap_ = populateAddressMap(bankMode);
}

void MCPDevice::initIntrGPIOPins(gpio_int_type_t modeA, gpio_int_type_t modeB) {}

void MCPDevice::setupDefaultIntterupt(INTR_TYPE type, INTR_OUTPUT_TYPE outtype,
									  PairedInterrupt sharedIntr)
{
	interruptManager_->setup(type, outtype, sharedIntr);
}

void MCPDevice::setIntteruptPin(PORT port, uint8_t pinmask, uint8_t mcpIntrmode,
								INTR_OUTPUT_TYPE intrOutMode)
{

	updateInterruptSetting(mcpIntrmode, intrOutMode);
	interruptManager_->setupInterruptMask(port, pinmask);
	interruptManager_->setup(intrSetting_);
}

void MCPDevice::updateInterruptSetting(uint8_t mcpIntrmode, INTR_OUTPUT_TYPE intrOutMode)
{
	intrSetting_.intrOutputType = intrOutMode;
	switch(mcpIntrmode)
	{
		case CHANGE:
			intrSetting_.intrType = INTR_TYPE::INTR_ON_CHANGE;
			break;
		case RISING:
			intrSetting_.intrType = INTR_TYPE::INTR_ON_RISING;
			break;
		case FALLING:
			intrSetting_.intrType = INTR_TYPE::INTR_ON_FALLING;
			break;
		default:
			break;
	}
}

void MCPDevice::attachInterrupt(gpio_num_t pinA, void (*intAHandler)(void*), uint8_t espIntrmode)
{
	intrSetting_.intrSharing = true;
	intrSetting_.modeA_ = coverIntrMode(espIntrmode);
	interruptManager_->setup(intrSetting_);
	interruptManager_->attachMainHandler<void>(PORT::GPIOA, pinA, intAHandler, nullptr);
}

void MCPDevice::attachInterrupt(gpio_num_t pinA, void (*intAHandler)(void*), uint8_t espIntrmodeA,
								gpio_num_t pinB, uint8_t espIntrmodeB, void (*intBHandler)(void*))
{
	if(pinA != static_cast<gpio_num_t>(-1) && pinB != static_cast<gpio_num_t>(-1))
	{
		intrSetting_.intrSharing = false;
	}
	intrSetting_.modeA_ = coverIntrMode(espIntrmodeA);
	intrSetting_.modeB_ = coverIntrMode(espIntrmodeB);
	interruptManager_->setup(intrSetting_);
	interruptManager_->attachMainHandler<void>(PORT::GPIOA, pinA, intAHandler, nullptr);
	interruptManager_->attachMainHandler<void>(PORT::GPIOB, pinB, intBHandler, nullptr);
}

gpio_int_type_t MCPDevice::coverIntrMode(uint8_t mode)
{

	gpio_int_type_t intMode = gpio_int_type_t::GPIO_INTR_DISABLE;
	switch(mode)
	{
		case CHANGE:
			intMode = gpio_int_type_t::GPIO_INTR_ANYEDGE;
			break;
		case RISING:
			intMode = gpio_int_type_t::GPIO_INTR_POSEDGE;
			break;
		case FALLING:
			intMode = gpio_int_type_t::GPIO_INTR_NEGEDGE;
			break;
		default:
			break;
	}
	return intMode;
}
bool MCPDevice::verifyDeviceOperation()
{
	LOG::INFO(MCP_TAG, "Checking Error Recovery");

	auto reg = getGPIORegister(REG::IODIR, PORT::GPIOA);
	uint8_t lastSavedValue = reg->getSavedValue();

	int value = i2cBus_.read_mcp_register(reg->getAddress(), bankMode_);
	if(value == -1)
	{
		LOG::ERROR(MCP_TAG, "Device verification failed - read returned -1");
		return false;
	}

	// Try writing to a test register
	uint8_t testValue = 0x1F;
	int writeResult = i2cBus_.write_mcp_register(reg->getAddress(), testValue, bankMode_);
	if(writeResult == -1)
	{
		LOG::ERROR(MCP_TAG, "Device verification failed - write failed");
		return false;
	}

	// Read back and verify
	value = i2cBus_.read_mcp_register(reg->getAddress(), bankMode_);
	if(value != testValue)
	{
		LOG::ERROR(MCP_TAG, "Device verification failed - read back mismatch: %02X vs %02X", value,
				   testValue);
		return false;
	}

	i2cBus_.write_mcp_register(reg->getAddress(), lastSavedValue, bankMode_);

	LOG::SUCCESS(MCP_TAG, "Device verification successful");
	return true;
}
void MCPDevice::dumpRegisters() const
{
	LOG::INFO(MCP_TAG, "Dumping Registers for MCP_Device (Address: 0x%02X)", address_);
	bool is16Bit = !bankMode_; // True if in 16-bit mode
	if(is16Bit)
	{
		LOG::INFO(MCP_TAG, "Register mapping: 16-bit mode (PORTA & PORTB separated)");

		for(uint8_t i = 0; i < MAX_REG_PER_PORT; i++)
		{
			REG regA = static_cast<REG>(i);
			REG regB = static_cast<REG>(i);

			uint8_t addressA = getRegisterAddress(regA, PORT::GPIOA);
			uint8_t valueA = i2cBus_.read_mcp_register(addressA, true);
			uint8_t valueB = i2cBus_.read_mcp_register(addressA + 1, true);

			LOG::INFO(MCP_TAG, "Register: %s (PORTA), Address: 0x%02X, Value: 0x%02X",
					  Util::ToString::REG(regA), addressA, valueA);

			LOG::INFO(MCP_TAG, "Register: %s (PORTB), Address: 0x%02X, Value: 0x%02X",
					  Util::ToString::REG(regB), addressA + 1, valueB);
		}
	}
	else
	{
		LOG::INFO(MCP_TAG, "Register mapping: 8-bit mode");

		// Read PORTA Registers
		LOG::INFO(MCP_TAG, "PORTA:");
		for(uint8_t i = 0; i < MAX_REG_PER_PORT; i++)
		{
			REG reg = static_cast<REG>(i);
			uint8_t address = getRegisterAddress(reg, PORT::GPIOA);
			uint8_t value = i2cBus_.read_mcp_register(address, bankMode_);
			ESP_LOGI(MCP_TAG, "Register: %s, Address: 0x%02X, Value: 0x%02X",
					 Util::ToString::REG(reg), address, value);
		}

		// Read PORTB Registers
		LOG::INFO(MCP_TAG, "PORTB:");
		for(uint8_t i = 0; i < MAX_REG_PER_PORT; i++)
		{
			REG reg = static_cast<REG>(i);
			uint8_t address = getRegisterAddress(reg, PORT::GPIOB);
			uint8_t value = i2cBus_.read_mcp_register(address, bankMode_);
			ESP_LOGI(MCP_TAG, "Register: %s, Address: 0x%02X, Value: 0x%02X",
					 Util::ToString::REG(reg), address, value);
		}
	}
}
std::pair<PORT, uint8_t> MCPDevice::getPortAndMask(int pin)
{
	assert(pin >= 0 && pin <= 15 && "Invalid pin");
	PIN pinEnum = static_cast<PIN>(pin);
	PORT port = Util::getPortFromPin(pinEnum);
	uint8_t mask = 1 << (static_cast<uint8_t>(pinEnum) % 8);
	return {port, mask};
}

} // namespace MCP
