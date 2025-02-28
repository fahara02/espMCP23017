#include "CORE/interruptManager.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace MCP
{
SemaphoreHandle_t InterruptManager::portATrigger = NULL;
SemaphoreHandle_t InterruptManager::portBTrigger = NULL;
InterruptManager::ISRHandlerData InterruptManager::isrHandlerDataA = {};
InterruptManager::ISRHandlerData InterruptManager::isrHandlerDataB = {};
InterruptManager::RetryInfo InterruptManager::retryA = {};
InterruptManager::RetryInfo InterruptManager::retryB = {};

InterruptManager::InterruptManager(MCP::MCP_MODEL m, I2CBus& bus,
								   std::shared_ptr<MCP::Register> iconA,
								   std::shared_ptr<MCP::Register> iconB) :
	model(m), i2cBus_(bus), regA(InterruptRegisters(iconA)), regB(InterruptRegisters(iconB))
{

	regA.setup(model, PORT::GPIOA, bankMode);
	regB.setup(model, PORT::GPIOA, bankMode);
	init(this);
}
void InterruptManager::init(InterruptManager* manager)
{
	bool initialised = false;
	if(manager)
	{
		if(!initialised)
		{
			if(isrMode == ISRMode::Default)
			{
				portATrigger = xSemaphoreCreateBinary();
				portBTrigger = xSemaphoreCreateBinary();
				xTaskCreatePinnedToCore(InterruptProcessorTask, "interruptMonitorTask", 4196,
										manager, 5, &intrTaskHandle, 1);
				initialised = true;
			}

			if(portATrigger != nullptr && portBTrigger != nullptr)
			{
				initialised = true;
			}
		}
	}
}
void InterruptManager::setupISR(gpio_num_t pin, ISRHandlerData& handlerData, gpio_int_type_t mode,
								void (*defaultHandler)(void*))
{
	gpio_config_t io_conf = {};
	io_conf.pin_bit_mask = (1ULL << pin);
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.intr_type = mode;

	gpio_config(&io_conf);

	gpio_isr_handler_add(pin, handlerData.handler ? isrWrapper : defaultHandler, &handlerData);
}

bool InterruptManager::initIntrGPIOPins()
{
	if(!isEnable())
		return false;

	static bool isrServiceInstalled = false;
	if(!isrServiceInstalled)
	{
		gpio_install_isr_service(0);
		isrServiceInstalled = true;
	}

	setupISR(static_cast<gpio_num_t>(pinA_), isrHandlerDataA, setting_.modeA_, defaultIntAHandler);
	setupISR(static_cast<gpio_num_t>(pinB_), isrHandlerDataB, setting_.modeB_, defaultIntBHandler);

	return true;
}

void InterruptManager::setup(INTR_TYPE type, INTR_OUTPUT_TYPE outtype, PairedInterrupt sharedIntr)
{

	setting_.intrType = type;
	setting_.intrOutputType = outtype;
	setting_.intrSharing = sharedIntr == PairedInterrupt::Enabled ? true : false;
	ESP_LOGI(INT_TAG, "setting loaded");
}
void InterruptManager::setup(InterruptSetting& setting)
{

	setting_.intrType = setting.intrType;
	setting_.intrOutputType = setting.intrOutputType;
	setting_.intrSharing = setting.intrSharing;
	setting_.modeA_ = setting.modeA_;
	setting_.modeB_ = setting.modeB_;
	ESP_LOGI(INT_TAG, "setting loaded");
}

bool InterruptManager::updateBankMode(bool value)
{
	// icon register not need to be invoked again as this method is already
	// invoked and icon is updated  ,just notify this class for updating address
	bankMode = value;
	regA.updateAddress(bankMode);
	regB.updateAddress(bankMode);
	return true;
}
bool InterruptManager::isEnable() { return setting_.isEnabled; }
bool InterruptManager::isSharing() { return setting_.intrSharing; }
uint8_t InterruptManager::getMask(PORT p) const { return p == PORT::GPIOA ? maskA_ : maskB_; }

void InterruptManager::updateMask(MCP::PORT port, uint8_t mask)
{
	if(port == MCP::PORT::GPIOA)
	{
		maskA_ |= mask;
	}
	else
	{
		maskB_ |= mask;
	}
}
void InterruptManager::setupInterruptMask(PORT port, uint8_t mask)
{
	if(port == PORT::GPIOA)
	{
		maskA_ = mask;
	}
	else
	{
		maskB_ = mask;
	}
}

Register* InterruptManager::getRegister(PORT port, REG reg)
{
	return (port == PORT::GPIOA ? regA : regB).getRegisterForUpdate(reg);
}
bool InterruptManager::updateRegisterValue(PORT port, uint8_t reg_address, uint8_t value)
{
	return (port == PORT::GPIOA ? regA : regB).updateRegisterValue(reg_address, value);
}

void InterruptManager::clearInterrupt()
{
	i2cBus_.read_mcp_register(regA.getAddress(REG::INTCAP), bankMode);
	i2cBus_.read_mcp_register(regB.getAddress(REG::INTCAP), bankMode);
}

bool InterruptManager::getInterruptFlags(uint8_t& flagA, uint8_t& flagB)
{
	int result = 0;

	if(!bankMode)
	{
		// 16bit Map mode
		int value = i2cBus_.read_mcp_register(regA.getAddress(REG::INTF), bankMode);
		flagA = static_cast<uint16_t>(value) & 0xFF; // extract lowByte
		flagB = (static_cast<uint16_t>(value) >> 8) & 0xFF; // extract highbyte
		result = value;

		return result != -1;
	}
	else
	{
		// 8bit Map mode
		result = i2cBus_.read_mcp_register(regA.getAddress(REG::INTF), bankMode);
		if(result == -1)
		{
			return false;
		}
		flagA = result;
		result = i2cBus_.read_mcp_register(regB.getAddress(REG::INTF), bankMode);
		if(result == -1)
		{
			return false;
		}
		flagB = result;

		return result != -1;
	}
}

bool InterruptManager::enableInterrupt()
{
	setting_.isEnabled = true;
	ESP_LOGI(INT_TAG, "enabling intterupt");
	if(updateInterrputSetting())
	{
		ESP_LOGI(INT_TAG, "SuccessFully intterrupt is set");
		return true;
	}
	else
	{
		ESP_LOGE(INT_TAG, "Error in intterupt setup");
		return false;
	}
}

bool InterruptManager::disabeInterrupt()
{
	setting_.isEnabled = false;
	return true;
}
bool InterruptManager::updateInterrputSetting()
{
	bool success = false;
	if(setting_.isEnabled)
	{
		if(setting_.intrSharing)
		{
			// setting mirror in just one ICON is enough
			regA.iocon->setInterruptSahring<REG::IOCON>(true);
			vTaskDelay(10);
			success =
				confirmRegisterIsSet(PORT::GPIOA, REG::IOCON, static_cast<uint8_t>(Field::MIRROR));
			vTaskDelay(10);
		}
		switch(setting_.intrType)
		{
			ESP_LOGI(INT_TAG, "checking intterupt types");
			case INTR_TYPE::INTR_ON_CHANGE:
				success = setupIntteruptOnChnage();

				return success;

			case INTR_TYPE::INTR_ON_RISING:
				success = setupIntteruptWithDefval(false);

				return success;

			case INTR_TYPE::INTR_ON_FALLING:
				success = setupIntteruptWithDefval(true);

				return success;

			case INTR_TYPE::NONE:
			default:
				return success;
		};
	}

	return success;
}
bool InterruptManager::resetInterruptRegisters()
{
	ESP_LOGI("MCP_DEVICE", "resetting all interrupts ");

	bool status = true;

	status &= (i2cBus_.write_mcp_register(regA.gpinten->getAddress(), 0x00, true) == 0);

	status &= (i2cBus_.write_mcp_register(regB.gpinten->getAddress(), 0x00, true) == 0);

	status &= (i2cBus_.write_mcp_register(regA.intcon->getAddress(), 0x00, true) == 0);
	status &= (i2cBus_.write_mcp_register(regB.intcon->getAddress(), 0x00, true) == 0);

	status &= (i2cBus_.read_mcp_register(regA.intcap->getAddress(), true) == 0);
	status &= (i2cBus_.read_mcp_register(regB.intcap->getAddress(), true) == 0);

	return status;
}

bool InterruptManager::setupIntteruptOnChnage()
{
	bool success = false;
	setting_.icoControl = INTR_ON_CHANGE_CONTROL::COMPARE_WITH_OLD_VALUE;
	// No change in INTCONA or B as it is already
	// default 0 Enable Intterupt
	success = setupEnableRegister();
	success = setupIntrOutput();
	return success;
}

bool InterruptManager::setupIntteruptWithDefval(bool savedValue)
{
	bool success = false;

	setting_.icoControl = INTR_ON_CHANGE_CONTROL::COMPARE_WITH_DEFVAL;
	// ACTIVATE defval comparison first
	regA.intcon->applyMask(maskA_);
	vTaskDelay(10);
	success = confirmRegisterIsSet(PORT::GPIOA, REG::INTCON, maskA_);
	vTaskDelay(10);
	if(!setting_.intrSharing)
	{
		regB.intcon->applyMask(maskB_);
		vTaskDelay(10);
		success = confirmRegisterIsSet(PORT::GPIOB, REG::INTCON, maskB_);
		vTaskDelay(10);
	}
	// Save the value to be compared with in DEFVAL
	regA.defval->saveCompareValue<REG::DEFVAL>(
		maskA_, savedValue ? DEF_VAL_COMPARE::SAVE_LOGIC_HIGH : DEF_VAL_COMPARE::SAVE_LOGIC_LOW);

	vTaskDelay(10);
	if(savedValue)
	{
		success = confirmRegisterIsSet(PORT::GPIOA, REG::DEFVAL, maskA_);
		vTaskDelay(10);
	}
	if(!setting_.intrSharing)
	{
		regB.defval->saveCompareValue<REG::DEFVAL>(maskB_, savedValue ?
															   DEF_VAL_COMPARE::SAVE_LOGIC_HIGH :
															   DEF_VAL_COMPARE::SAVE_LOGIC_LOW);
		vTaskDelay(10);
		if(savedValue)
		{
			success = confirmRegisterIsSet(PORT::GPIOB, REG::DEFVAL, maskB_);
			vTaskDelay(10);
		}
	}
	// Enable Intterupt
	success = setupEnableRegister();

	// Select Intterupt Output Type
	success = setupIntrOutput();
	return success;
}
bool InterruptManager::setupEnableRegister()
{
	bool success = false;

	regA.gpinten->applyMask(maskA_);
	vTaskDelay(10);
	success = confirmRegisterIsSet(PORT::GPIOA, REG::GPINTEN, maskA_);
	vTaskDelay(10);
	if(!setting_.intrSharing)
	{
		regB.gpinten->applyMask(maskB_);
		vTaskDelay(10);
		success = confirmRegisterIsSet(PORT::GPIOB, REG::GPINTEN, maskB_);
		vTaskDelay(10);
	}
	return success;
}
bool InterruptManager::setupIntrOutput()
{
	bool success = false;

	if(setting_.intrOutputType == INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH)
	{
		regA.iocon->setInterruptPolarity<REG::IOCON>(true);
		vTaskDelay(10);
		success =
			confirmRegisterIsSet(PORT::GPIOA, REG::IOCON, static_cast<uint8_t>(Field::INTPOL));
		vTaskDelay(10);
	}
	else if(setting_.intrOutputType == INTR_OUTPUT_TYPE::INTR_OPEN_DRAIN)
	{
		regA.iocon->setOpenDrain<REG::IOCON>(true);
		// No Need to change INTPOL as it is default 0
		vTaskDelay(10);
		success = confirmRegisterIsSet(PORT::GPIOA, REG::IOCON, static_cast<uint8_t>(Field::ODR));
		vTaskDelay(10);
		// Setting in ICONA will have same reflection on
		// ICONB so redundant
	}
	else
	{
		// No need to write and read IOCON intpol as it
		// is default 0
		success = true;
	}
	return success;
}

bool InterruptManager::confirmRegisterIsSet(PORT port, REG regType, uint8_t bitMask)
{
	bool success = false;

	uint8_t address = port == PORT::GPIOA ? regA.getAddress(regType) : regB.getAddress(regType);

	int currentValue = i2cBus_.read_mcp_register(address, bankMode);
	if(currentValue == -1)
	{
		ESP_LOGE(INT_TAG, "Error setting reg %s", Util::ToString::REG(regType));
	}
	else
	{

		if(Util::BIT::isSingleBit(bitMask))
		{
			bool checkedBit = Util::BIT::isSet(currentValue, bitMask);
			success = checkedBit;
		}
		else
		{
			bool chekedMask = Util::BIT::isAllSet(currentValue, bitMask);
			success = chekedMask;
		}
	}
	return success;
}

void IRAM_ATTR InterruptManager::defaultIntAHandler(void* arg)
{

	BaseType_t urgentTask = pdFALSE;
	xSemaphoreGiveFromISR(portATrigger, &urgentTask);
	if(urgentTask)
	{
		portYIELD_FROM_ISR(urgentTask);
	}
}

void IRAM_ATTR InterruptManager::defaultIntBHandler(void* arg)
{

	BaseType_t urgentTask = pdFALSE;
	xSemaphoreGiveFromISR(portBTrigger, &urgentTask);
	if(urgentTask)
	{
		portYIELD_FROM_ISR(urgentTask);
	}
}
void InterruptManager::InterruptProcessorTask(void* param)
{
	InterruptManager* manager = static_cast<InterruptManager*>(param);

	while(true)
	{

		if(xSemaphoreTake(portATrigger, portMAX_DELAY) || manager->hasflagReadAFailed())
		{

			uint8_t flagA = 0;
			uint8_t flagB = 0;
			bool result = manager->getInterruptFlags(flagA, flagB);

			if(!result)
			{
				Serial.printf("Port A: Interrupt flag read failed retrying for %d time...\n",
							  manager->getRetryA());
				manager->retryFlagReadA();

				if(manager->getRetryA() > MCP::MAX_RETRY)
				{
					manager->resetRetryA();
					Serial.println("Port A: Interrupt flag read retry limit reached !");
				}
				vTaskDelay(pdMS_TO_TICKS(10));
			}

			if(manager->isSharing())
			{

				manager->processPort(PORT::GPIOA, flagA);
				manager->processPort(PORT::GPIOB, flagB);
			}
			else
			{

				manager->processPort(PORT::GPIOA, flagA);
			}
			manager->clearInterrupt();
			vTaskDelay(pdMS_TO_TICKS(10));
		}
		if(xSemaphoreTake(portBTrigger, portMAX_DELAY) || manager->hasflagReadBFailed())
		{
			uint8_t flagA = 0;
			uint8_t flagB = 0;
			bool result = manager->getInterruptFlags(flagA, flagB);

			if(!result)
			{
				Serial.printf("Port B: Interrupt flag read failed retrying for %d time...\n",
							  manager->getRetryB());
				manager->retryFlagReadB();

				if(manager->getRetryB() > MCP::MAX_RETRY)
				{
					manager->resetRetryB();
					Serial.println("Port B: Interrupt flag read retry limit reached !");
				}
				vTaskDelay(pdMS_TO_TICKS(10));
			}

			if(manager->isSharing())
			{

				manager->processPort(PORT::GPIOA, flagA);
				manager->processPort(PORT::GPIOB, flagB);
			}
			else
			{

				manager->processPort(PORT::GPIOB, flagB);
			}
			manager->clearInterrupt();
			vTaskDelay(pdMS_TO_TICKS(10));
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
	vTaskDelete(NULL);
}

void IRAM_ATTR InterruptManager::processPort(MCP::PORT port, uint8_t flag)
{
	for(uint8_t i = 0; i < MCP::PIN_PER_BANK; ++i)
	{
		if(flag & (1 << i))
		{
			invokeCallback(port, i);
		}
	}
}

void InterruptManager::invokeCallback(MCP::PORT port, uint8_t pinindex)
{

	Callback cb;
	if(port == MCP::PORT::GPIOA)
	{
		cb = portACallbacks_[pinindex];
	}
	else
	{
		cb = portBCallbacks_[pinindex];
	}

	if(cb.fn)
	{
		cb.fn();
	}
}

} // namespace MCP
