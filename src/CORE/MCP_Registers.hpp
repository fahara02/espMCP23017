#ifndef MCP_REGISTERS_HPP
#define MCP_REGISTERS_HPP
#include "CORE/MCP_Constants.hpp"
#include "CORE/MCP_Primitives.hpp"
#include "CORE/RegisterEvents.hpp"
#include "Util/Utility.hpp"
#include "Util/Logger.hpp"
#include "freertos/semphr.h"
#include "memory"
#include <type_traits>
#include <unordered_map>

#define REG_TAG "MCP_REGISTERS"
namespace MCP
{
struct address_decoder_t
{
	const MCP::MCP_MODEL model_;
	const bool A2_;
	const bool A1_;
	const bool A0_;

	address_decoder_t(MCP::MCP_MODEL m, bool A2, bool A1, bool A0) :
		model_(m), A2_(A2), A1_(A1), A0_(A0)
	{
	}

	uint8_t getDeviceAddress(bool haen) { return decodeDeviceAddress(haen); }

  private:
	constexpr uint8_t decodeDeviceAddress(bool haen)
	{
		if(model_ == MCP::MCP_MODEL::MCP23017 || (model_ == MCP::MCP_MODEL::MCP23S17 && haen))
		{
			return MCP_ADDRESS_BASE | (A2_ << 2) | (A1_ << 1) | A0_;
		}
		return MCP_ADDRESS_BASE;
	}
};
struct Settings
{
	OperationMode opMode = OperationMode::SequentialMode16;
	PairedInterrupt mirror = PairedInterrupt::Disabled;
	Slew slew = Slew::Enabled;
	HardwareAddr haen = HardwareAddr::Disabled;
	OpenDrain odr = OpenDrain::Disabled;
	InterruptPolarity intpol = InterruptPolarity::ActiveLow;

	explicit Settings(MCP::MCP_MODEL m = MCP::MCP_MODEL::MCP23017) : model_(m) {}

	bool operator==(const Settings& other) const
	{
		return opMode == other.opMode && mirror == other.mirror && slew == other.slew &&
			   haen == other.haen && odr == other.odr && intpol == other.intpol &&
			   model_ == other.model_;
	}

	bool operator!=(const Settings& other) const { return !(*this == other); }

	uint8_t getSetting() const
	{
		return (static_cast<uint8_t>(opMode == OperationMode::SequentialMode8 ||
									 opMode == OperationMode::ByteMode8)
				<< 7) |
			   (static_cast<uint8_t>(mirror) << 6) |
			   (static_cast<uint8_t>(opMode == OperationMode::ByteMode16 ||
									 opMode == OperationMode::ByteMode8)
				<< 5) |
			   (static_cast<uint8_t>(slew) << 4) | (static_cast<uint8_t>(haen) << 3) |
			   (static_cast<uint8_t>(odr) << 2) | (static_cast<uint8_t>(intpol) << 1);
	}

	void setModel(MCP::MCP_MODEL m) { model_ = m; }
	MCP::MCP_MODEL getModel() const { return model_; }

  private:
	MCP::MCP_MODEL model_;
};

// ================= Config Struct ==================
struct Config
{
	enum Field : uint8_t
	{
		RESERVED,
		INTPOL,
		ODR,
		HAEN,
		DISSLW,
		SEQOP,
		MIRROR,
		BANK
	};

	explicit Config(MCP::MCP_MODEL m = MCP::MCP_MODEL::MCP23017) : config_(m), value(0) {}

	void configureDefault() { validate_update(config_); }

	bool configure(const Settings& setting)
	{
		Settings validated = setting;
		applyValidationRules(validated);
		return validate_update(validated);
	}

	bool configure(uint8_t new_setting) { return configure(extractSettings(new_setting)); }

	uint8_t getSettingValue() const { return value; }
	Settings getSettings() const { return config_; }

	bool getBitField(Field field) const { return value & (1 << field); }

	void setOperationMode(bool byteMode, bool mapping8Bit)
	{
		config_.opMode =
			byteMode ?
				(mapping8Bit ? OperationMode::ByteMode8 : OperationMode::ByteMode16) :
				(mapping8Bit ? OperationMode::SequentialMode8 : OperationMode::SequentialMode16);
		updateConfig();
	}

	void setMirror(bool enable) { updateSetting(config_.mirror, enable); }
	void setSlewRate(bool enable) { updateSetting(config_.slew, enable); }
	void setOpenDrain(bool enable) { updateSetting(config_.odr, enable); }
	void setInterruptPolarity(bool activeHigh) { updateSetting(config_.intpol, activeHigh); }

	void setHardwareAddressing(bool enable)
	{
		if(config_.getModel() == MCP_MODEL::MCP23017 || config_.getModel() == MCP_MODEL::MCP23018)
		{
			enable = false;
		}
		updateSetting(config_.haen, enable);
	}

  private:
	Settings config_;
	uint8_t value;

	void updateConfig() { value = config_.getSetting(); }

	void applyValidationRules(Settings& setting)
	{
		if(setting.getModel() == MCP_MODEL::MCP23017 || setting.getModel() == MCP_MODEL::MCP23018)
		{
			setting.haen = HardwareAddr::Disabled;
		}
		if(setting.odr == OpenDrain::Enabled && setting.intpol == InterruptPolarity::ActiveHigh)
		{
			setting.intpol = InterruptPolarity::ActiveLow;
		}
	}

	bool validate_update(Settings& setting)
	{
		if(config_ == setting)
			return false;
		config_ = setting;
		updateConfig();
		return true;
	}

	Settings extractSettings(uint8_t setting)
	{
		Settings new_config(config_.getModel());
		new_config.opMode =
			(setting & (1 << 7)) ?
				((setting & (1 << 5)) ? OperationMode::ByteMode8 : OperationMode::SequentialMode8) :
				((setting & (1 << 5)) ? OperationMode::ByteMode16 :
										OperationMode::SequentialMode16);
		new_config.mirror = static_cast<PairedInterrupt>(setting & (1 << 6));
		new_config.slew = static_cast<Slew>(!(setting & (1 << 4)));
		new_config.haen = static_cast<HardwareAddr>(setting & (1 << 3));
		new_config.odr = static_cast<OpenDrain>(setting & (1 << 2));
		new_config.intpol = static_cast<InterruptPolarity>(setting & (1 << 1));

		return new_config;
	}

	template<typename T>
	void updateSetting(T& field, bool enable)
	{
		if(field != static_cast<T>(enable))
		{
			field = static_cast<T>(enable);
			updateConfig();
		}
	}
};

struct Register
{
	using configField = MCP::Config::Field;
	const MCP_MODEL model;
	const MCP::REG reg;
	const MCP::PORT port;
	Register(MCP_MODEL m = MCP_MODEL::MCP23017, MCP::REG rg = MCP::REG::IODIR,
			 MCP::PORT p = MCP::PORT::GPIOA, bool bankMode = false, bool readonly = false) :
		model(m), reg(rg), port(p), readOnly_(readonly), bankSeparated_(bankMode), value_(0),
		regAddress_(Util::calculateAddress(reg, port, bankSeparated_)),
		identity_(reg, port, regAddress_), config_(Config(m))
	{
		init();
	}

	void init()
	{

		if(reg == REG::IOCON)
		{
			value_ = 0X00;
			config_.configureDefault();
		}
		else if(reg == REG::IODIR)
		{

			value_ = 0XFF;
		}
		else
		{
			value_ = 0X00;
		}
	}

	void updateBankMode(bool bankMode)
	{
		bankSeparated_ = bankMode;
		regAddress_ = Util::calculateAddress(reg, port, bankSeparated_);
		identity_.regAddress = regAddress_;
	}
	uint8_t getSavedValue() const { return reg == REG::IOCON ? config_.getSettingValue() : value_; }
	uint8_t getSavedSettings() const { return config_.getSettingValue(); }
	void updateState(currentEvent& ev)
	{
		if(ev.regIdentity.regAddress == regAddress_)
		{
			setValue(ev.data);
			if(reg == REG::IOCON)
			{
				config_.configure(ev.data);
			}
		}
	}
	bool updateState(uint8_t newValue)
	{
		if(reg == REG::IOCON)
		{
			return config_.configure(newValue);
		}
		else
		{

			value_ = newValue;
			return true;
		}
	}
	void setValue(uint8_t newvalue, bool intrFun = false)
	{
		if(!readOnly_)
		{

			value_ = newvalue;
			if(reg == MCP::REG::IOCON)
			{
				EventManager::createEvent(identity_, RegisterEvent::SETTINGS_CHANGED, value_,
										  intrFun);
			}
			else
			{
				EventManager::createEvent(identity_, RegisterEvent::WRITE_REQUEST, value_, intrFun);
			}
		}
	}

	void setBitField(uint8_t bit, bool bit_value, bool intrFun = false)
	{

		if(!readOnly_)
		{
			if(bit < 8)
			{
				if(bit_value)
				{
					Util::BIT::set(value_, bit);
				}
				else
				{
					Util::BIT::clear(value_, bit);
				}
			}

			LOG::INFO(REG_TAG, "Bit field %d set to %d in register 0x%02X", bit, value_,
					  regAddress_);
			if(reg == MCP::REG::IOCON)
			{
				EventManager::createEvent(identity_, RegisterEvent::SETTINGS_CHANGED, value_,
										  intrFun);
			}
			else
			{
				EventManager::createEvent(identity_, RegisterEvent::WRITE_REQUEST, value_, intrFun);
			}
		}
	}
	uint8_t getValue(bool intrFun = false)
	{

		EventManager::createEvent(identity_, RegisterEvent::READ_REQUEST, 0, intrFun);

		EventBits_t bits = xEventGroupWaitBits(
			EventManager::registerEventGroup,
			static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED), pdFALSE, pdFALSE, READ_TIMEOUT);

		if(!(bits & static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED)))
		{
			LOG::ERROR(REG_TAG, "Timeout waiting for DATA_RECEIVED event!");
			EventManager::createEvent(identity_, RegisterEvent::ERROR, config_.getSettingValue());
			LOG::ERROR(REG_TAG, "Error Event Created in getValue!");
			EventManager::createEvent(identity_, RegisterEvent::DATA_RECEIVED, 0xFF);
			EventManager::clearBits(RegisterEvent::DATA_RECEIVED);
			return 0xFF;
		}
		currentEvent* event = EventManager::getEvent(RegisterEvent::DATA_RECEIVED);
		EventManager::acknowledgeEvent(event);
		EventManager::clearBits(RegisterEvent::DATA_RECEIVED);

		return value_;
	}
	bool getBitField(uint8_t bit, bool intrFun = false)
	{
		EventManager::createEvent(identity_, RegisterEvent::READ_REQUEST, intrFun);

		EventBits_t bits = xEventGroupWaitBits(
			EventManager::registerEventGroup,
			static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED), pdFALSE, pdFALSE, READ_TIMEOUT);

		if(!(bits & static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED)))
		{
			LOG::ERROR(REG_TAG, "Timeout waiting for DATA_RECEIVED event for getField!");
			EventManager::createEvent(identity_, RegisterEvent::ERROR, config_.getSettingValue());
			LOG::ERROR(REG_TAG, "Error Event Created in GetBitField!");
			return 0xFF;
		}
		currentEvent* event = EventManager::getEvent(RegisterEvent::DATA_RECEIVED);
		EventManager::acknowledgeEvent(event);
		EventManager::clearBits(RegisterEvent::DATA_RECEIVED);
		return (bit < 8) ? Util::BIT::isSet(value_, bit) : false;
	}
	void applyMask(uint8_t mask, bool intrFun = false)
	{

		if(reg != REG::IOCON)
		{
			value_ |= mask;
			LOG::INFO(REG_TAG, "New Value %d set  in register 0x%02X", value_, regAddress_);
			EventManager::createEvent(identity_, RegisterEvent::WRITE_REQUEST, value_, intrFun);
		}
	}

	void clearMask(uint8_t mask, bool intrFun = false)
	{
		if(reg != REG::IOCON)
		{
			value_ &= ~mask;
			LOG::INFO(REG_TAG, "New Value %d set  in register 0x%02X", value_, regAddress_);

			EventManager::createEvent(identity_, RegisterEvent::WRITE_REQUEST, value_, intrFun);
		}
	}
	void setAsReadOnly() { readOnly_ = true; }
	uint8_t getAddress() const { return regAddress_; }
	registerIdentity getIdentity() const { return identity_; }

	template<REG T>
	typename std::enable_if<T == REG::IOCON, bool>::type configure(const uint8_t& settings)
	{
		return config_.configure(settings);
	};
	template<REG T>
	typename std::enable_if<T == REG::IOCON, bool>::type configure(const Settings& setting)
	{
		return config_.configure(setting);
	}

	template<REG T>
	typename std::enable_if<T == REG::IOCON, void>::type setInterruptSahring(bool enable)
	{
		config_.setMirror(enable);
		EventManager::createEvent(identity_, RegisterEvent::SETTINGS_CHANGED,
								  config_.getSettingValue());
	}
	template<REG T>
	typename std::enable_if<T == REG::IOCON, bool>::type setOpenDrain(bool enable)
	{
		bool intPolMode = config_.getBitField(configField::INTPOL);

		if(!intPolMode && enable)
		{
			config_.setOpenDrain(true);
			config_.setInterruptPolarity(false);
			EventManager::createEvent(identity_, RegisterEvent::SETTINGS_CHANGED,
									  config_.getSettingValue());
			return true;
		}
		else if(intPolMode && enable)
		{
			config_.setOpenDrain(false);
			EventManager::createEvent(identity_, RegisterEvent::SETTINGS_CHANGED,
									  config_.getSettingValue());
			return true;
		}
		return false;
	}
	template<REG T>
	typename std::enable_if<T == REG::IOCON, bool>::type setInterruptPolarity(bool activeHigh)
	{
		bool outputMode = config_.getBitField(configField::ODR);
		if(!outputMode)
		{
			config_.setInterruptPolarity(activeHigh);
			EventManager::createEvent(identity_, RegisterEvent::SETTINGS_CHANGED,
									  config_.getSettingValue());
			return true;
		}
		return false;
	}

	// INTERRUPT SPECIFIC
	template<REG T>
	typename std::enable_if<T == REG::GPINTEN, bool>::type
		setInterruptOnChange(PIN pin, INTR_ON_CHANGE_ENABLE en)
	{

		if(port != Util::getPortFromPin(pin))
		{
			LOG::ERROR(REG_TAG, "Pin %d does not belong to the expected port",
					   static_cast<int>(pin));
			EventManager::createEvent(identity_, RegisterEvent::ERROR, config_.getSettingValue());
			return false;
		}
		uint8_t index = Util::getPinIndex(pin);
		setBitField(index, en == INTR_ON_CHANGE_ENABLE::ENABLE_INTR_ON_CHANGE, true);

		return true;
	}
	template<REG T>
	typename std::enable_if<T == REG::GPINTEN, bool>::type
		setInterruptOnChange(uint8_t pinmask, INTR_ON_CHANGE_ENABLE en)
	{

		if(en == INTR_ON_CHANGE_ENABLE::ENABLE_INTR_ON_CHANGE)
		{
			applyMask(pinmask, true);
		}
		else
		{
			clearMask(pinmask, true);
		}
		return true;
	}
	template<REG T>
	typename std::enable_if<T == REG::INTCON, bool>::type
		setInterruptType(PIN pin, INTR_ON_CHANGE_CONTROL cntrl)
	{

		if(port != Util::getPortFromPin(pin))
		{
			LOG::ERROR(REG_TAG, "Pin %d does not belong to the expected port",
					   static_cast<int>(pin));
			return false;
		}
		uint8_t index = Util::getPinIndex(pin);
		setBitField(index, cntrl == INTR_ON_CHANGE_CONTROL::COMPARE_WITH_DEFVAL, true);

		return true;
	}
	template<REG T>
	typename std::enable_if<T == REG::INTCON, bool>::type
		setInterruptType(uint8_t pinmask, INTR_ON_CHANGE_CONTROL cntrl)
	{

		if(cntrl == INTR_ON_CHANGE_CONTROL::COMPARE_WITH_DEFVAL)
		{
			applyMask(pinmask, true);
		}
		else
		{
			clearMask(pinmask, true);
		}

		return true;
	}
	template<REG T>
	typename std::enable_if<T == REG::DEFVAL, bool>::type saveCompareValue(PIN pin,
																		   DEF_VAL_COMPARE cmp)
	{
		if(port != Util::getPortFromPin(pin))
		{
			LOG::ERROR(REG_TAG, "Pinmask does not belong to the expected port");
			return false;
		}
		uint8_t index = Util::getPinIndex(pin);
		setBitField(index, cmp == DEF_VAL_COMPARE::SAVE_LOGIC_HIGH, true);

		return true;
	}
	template<REG T>
	typename std::enable_if<T == REG::DEFVAL, bool>::type saveCompareValue(uint8_t pinmask,
																		   DEF_VAL_COMPARE cmp)
	{

		if(cmp == DEF_VAL_COMPARE::SAVE_LOGIC_HIGH)
		{
			applyMask(pinmask);
		}
		else
		{
			clearMask(pinmask);
		}

		return true;
	}
	template<REG T>
	typename std::enable_if<T == REG::INTF, bool>::type readFlag()
	{

		EventManager::createEvent(identity_, RegisterEvent::READ_REQUEST, 0, true);

		EventBits_t bits = xEventGroupWaitBits(
			EventManager::registerEventGroup,
			static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED), pdFALSE, pdFALSE, READ_TIMEOUT);

		if(!(bits & static_cast<EventBits_t>(RegisterEvent::DATA_RECEIVED)))
		{
			LOG::ERROR(REG_TAG, "Timeout waiting for DATA_RECEIVED event!");
			EventManager::createEvent(identity_, RegisterEvent::ERROR, config_.getSettingValue());
			return 0xFF;
		}
		currentEvent* event = EventManager::getEvent(RegisterEvent::DATA_RECEIVED);
		EventManager::acknowledgeEvent(event);
		EventManager::clearBits(RegisterEvent::DATA_RECEIVED);

		return value_;
	}
	template<REG T>
	typename std::enable_if<T == REG::INTCAP, bool>::type clearInterrupt()
	{

		if(getValue())
		{
			return true;
		}
		else
		{
			return false;
		}
	}

  private:
	bool readOnly_ = false;
	bool bankSeparated_ = false;
	uint8_t value_;
	uint8_t regAddress_;
	registerIdentity identity_;
	Config config_;
};

class RegistersCollection
{
  protected:
	virtual std::unordered_map<MCP::REG, std::unique_ptr<MCP::Register>>& getRegMap() = 0;
	virtual const std::unordered_map<MCP::REG, std::unique_ptr<MCP::Register>>&
		getRegMap() const = 0;

  public:
	RegistersCollection(std::shared_ptr<MCP::Register> icon) { iocon = icon; }
	RegistersCollection(const RegistersCollection&) = delete;
	RegistersCollection& operator=(const RegistersCollection&) = delete;

	std::shared_ptr<MCP::Register> iocon;
	virtual void setup(MCP::MCP_MODEL model, MCP::PORT port, bool bankMode) = 0;
	virtual void assign() = 0;
	virtual bool updateRegisterValue(uint8_t reg_address, uint8_t value) = 0;
	void updateAddress(bool bankMode)
	{
		if(iocon)
		{
			iocon->updateBankMode(bankMode);
		}
		for(auto& [regType, regPtr]: getRegMap())
		{
			regPtr->updateBankMode(bankMode);
		}
	}
	std::shared_ptr<MCP::Register> const getIOCON() { return iocon; }
	// Get a register (read-only) by its type.
	const MCP::Register* getRegister(MCP::REG regType) const
	{
		if(regType == MCP::REG::IOCON)
			return iocon.get();
		auto it = getRegMap().find(regType);
		return (it != getRegMap().end()) ? it->second.get() : nullptr;
	}

	// Get a register (for update) by its type.
	MCP::Register* getRegisterForUpdate(MCP::REG regType)
	{
		if(regType == MCP::REG::IOCON)
			return iocon.get();
		auto it = getRegMap().find(regType);
		return (it != getRegMap().end()) ? it->second.get() : nullptr;
	}

	// Retrieve the saved value from a given register.
	uint8_t getSavedValue(MCP::REG reg) const
	{
		if(reg == MCP::REG::IOCON)
		{
			return iocon->getSavedValue();
		}
		else
		{
			auto it = getRegMap().find(reg);
			return (it != getRegMap().end()) ? it->second->getSavedValue() : 0;
		}
	}

	// Retrieve the address of a given register.
	uint8_t getAddress(MCP::REG reg) const
	{
		if(reg == MCP::REG::IOCON)
		{
			return iocon->getAddress();
		}
		else
		{
			auto it = getRegMap().find(reg);
			return (it != getRegMap().end()) ? it->second->getAddress() : 0;
		}
	}

	virtual ~RegistersCollection() = default;
};

// =================== GPIORegisters Class ===================

class GPIORegisters : public RegistersCollection
{
  private:
	// Register map for GPIO-related registers.
	std::unordered_map<MCP::REG, std::unique_ptr<MCP::Register>> regMap;

  public:
	GPIORegisters(std::shared_ptr<MCP::Register> icon) : RegistersCollection(icon) {}
	// Raw pointers for easier access to GPIO registers.
	MCP::Register* iodir{};
	MCP::Register* gppu{};
	MCP::Register* ipol{};
	MCP::Register* gpio{};
	MCP::Register* olat{};

	GPIORegisters() = default;

	// Provide access to the register map.
	std::unordered_map<MCP::REG, std::unique_ptr<MCP::Register>>& getRegMap() override
	{
		return regMap;
	}
	const std::unordered_map<MCP::REG, std::unique_ptr<MCP::Register>>& getRegMap() const override
	{
		return regMap;
	}

	bool updateRegisterValue(uint8_t reg_address, uint8_t value) override
	{
		if(iocon->getAddress() == reg_address)
		{
			return iocon->updateState(value);
		}

		for(auto& [regType, regPtr]: regMap)
		{
			if(regPtr->getAddress() == reg_address)
			{
				return regPtr->updateState(value);
			}
		}

		ESP_LOGE("MCPRegisters", "Failed to update register: Invalid address (0x%02X)",
				 reg_address);
		return false;
	}

	// Override setup: Create the IOCON register and the GPIO-specific registers.
	void setup(MCP::MCP_MODEL model, MCP::PORT port, bool bankMode) override
	{
		// Create the common IOCON register.

		// Create the GPIO registers.
		for(auto regType:
			{MCP::REG::IODIR, MCP::REG::GPPU, MCP::REG::IPOL, MCP::REG::GPIO, MCP::REG::OLAT})
		{
			regMap[regType] = std::make_unique<MCP::Register>(model, regType, port, bankMode);
		}
		assign();
	}

	// Override assign: Set raw pointers from the register map.
	void assign() override
	{
		iodir = regMap[MCP::REG::IODIR].get();
		gppu = regMap[MCP::REG::GPPU].get();
		ipol = regMap[MCP::REG::IPOL].get();
		gpio = regMap[MCP::REG::GPIO].get();
		olat = regMap[MCP::REG::OLAT].get();
	}
};

// =================== InterruptRegisters Class ===================

class InterruptRegisters : public RegistersCollection
{
  private:
	std::unordered_map<MCP::REG, std::unique_ptr<MCP::Register>> regMap;

  public:
	InterruptRegisters(std::shared_ptr<MCP::Register> icon) :
		RegistersCollection(icon) {} // Raw pointers for easier access to the interrupt registers.
	MCP::Register* gpinten{};
	MCP::Register* intcon{};
	MCP::Register* defval{};
	MCP::Register* intf{};
	MCP::Register* intcap{};

	InterruptRegisters() = default;

	// Provide access to the register map.
	std::unordered_map<MCP::REG, std::unique_ptr<MCP::Register>>& getRegMap() override
	{
		return regMap;
	}
	const std::unordered_map<MCP::REG, std::unique_ptr<MCP::Register>>& getRegMap() const override
	{
		return regMap;
	}
	bool updateRegisterValue(uint8_t reg_address, uint8_t value) override
	{
		if(iocon->getAddress() == reg_address)
		{
			return iocon->updateState(value);
		}

		for(auto& [regType, regPtr]: regMap)
		{
			if(regPtr->getAddress() == reg_address)
			{
				return regPtr->updateState(value);
			}
		}

		ESP_LOGE("MCPRegisters", "Failed to update register: Invalid address (0x%02X)",
				 reg_address);
		return false;
	}

	// registers.
	void setup(MCP::MCP_MODEL model, MCP::PORT port, bool bankMode) override
	{

		// Create the interrupt registers.
		for(auto regType: {MCP::REG::GPINTEN, MCP::REG::INTCON, MCP::REG::DEFVAL, MCP::REG::INTF,
						   MCP::REG::INTCAP})
		{
			regMap[regType] = std::make_unique<MCP::Register>(model, regType, port, bankMode);
		}
		assign();
	}

	// Override assign: Set raw pointers from the register map.
	void assign() override
	{
		gpinten = regMap[MCP::REG::GPINTEN].get();
		intcon = regMap[MCP::REG::INTCON].get();
		defval = regMap[MCP::REG::DEFVAL].get();
		intf = regMap[MCP::REG::INTF].get();
		intcap = regMap[MCP::REG::INTCAP].get();

		if(intf)
		{
			intf->setAsReadOnly();
		}
	}
};

} // namespace MCP
#endif
