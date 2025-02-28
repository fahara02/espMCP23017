#ifndef MCP_GPIO_BANKS_HPP
#define MCP_GPIO_BANKS_HPP
#include "MCP_Primitives.hpp"
#include "MCP_Registers.hpp"
#include "esp_log.h"
#include "i2cBus.hpp"
#include "memory"
#include <unordered_map>

#define BANK_TAG "GPIO_BANK"
namespace MCP {

class GPIO_BANK {
private:
  std::array<Pin, PIN_PER_BANK> Pins;
  GPIORegisters regs;

public:
  GPIO_BANK(PORT port, MCP::MCP_MODEL m, std::shared_ptr<MCP::Register> icon)
      : Pins(createPins(port)), regs(GPIORegisters(icon)), model(m),
        generalMask(0XFF), port_name(port) {

    regs.setup(model, port_name, bankMode);
    init();
  }

  std::shared_ptr<MCP::Register> const getControlRegister() {
    return regs.getIOCON();
  }
  const Register *getRegister(MCP::REG regType) const {
    return regs.getRegister(regType);
  }
  Register *getRegisterForUpdate(MCP::REG regType) {
    return regs.getRegisterForUpdate(regType);
  }

  uint8_t getAddress(REG reg) const { return regs.getAddress(reg); }

  bool updateBankMode(bool value) {
    // icon register not need to be invoked again as this method is already
    // invoked and icon is updated  ,just notify this class for updating address
    bankMode = value;
    regs.updateAddress(bankMode);
    return true;
  }
  bool updateRegisterValue(uint8_t reg_address, uint8_t value) {
    return regs.updateRegisterValue(reg_address, value);
  }

  // This method will not trigger any read event , only return register's
  // current saved value
  uint8_t getSavedValue(REG reg) const { return regs.getSavedValue(reg); }

  // Pin masks
  void setGeneralMask(MASK mask) { generalMask = static_cast<uint8_t>(mask); }
  void setGeneralMask(uint8_t mask) { generalMask = mask; }
  uint8_t getGeneralMask() const { return generalMask; }

  // PIN DIRECTION SELECTION
  void setPinDirection(PIN p, GPIO_MODE m) {
    assert(Util::getPortFromPin(p) == port_name && "Invalid pin ");
    regs.iodir->setBitField(static_cast<uint8_t>(p),
                            m == GPIO_MODE::GPIO_INPUT);
  }
  void setPinDirection(uint8_t pinmask, GPIO_MODE mode) {
    if (mode == GPIO_MODE::GPIO_INPUT) {
      regs.iodir->applyMask(pinmask);
    } else {
      regs.iodir->clearMask(pinmask);
    }
  }

  void setPinDirection(GPIO_MODE mode) {

    if (mode == GPIO_MODE::GPIO_INPUT) {
      regs.iodir->applyMask(generalMask);
    } else {
      regs.iodir->clearMask(generalMask);
    }
  }

  // PULLUP SELECTION
  void setPullup(PIN pin, PULL_MODE mode) {
    assert(Util::getPortFromPin(pin) == port_name && "Invalid pin ");
    regs.gppu->setBitField(static_cast<uint8_t>(pin),
                           mode == PULL_MODE::ENABLE_PULLUP);
  }

  void setPullup(uint8_t pinMask, PULL_MODE mode) {
    if (mode == PULL_MODE::ENABLE_PULLUP) {
      regs.gppu->applyMask(pinMask);
    } else {
      regs.gppu->clearMask(pinMask);
    }
  }

  void setPullup(PULL_MODE mode) {
    if (mode == PULL_MODE::ENABLE_PULLUP) {
      regs.gppu->applyMask(generalMask);
    } else {
      regs.gppu->clearMask(generalMask);
    }
  }

  // Get PIN VALUE
  bool getPinState(MCP::PIN p) {
    assert(Util::getPortFromPin(p) == port_name && "Invalid pin ");
    return regs.gpio->getBitField(static_cast<uint8_t>(p));
  }
  uint8_t getPinState(uint8_t pinmask) {

    uint8_t value = (regs.gpio->getValue() & pinmask);
    return value;
  }
  uint8_t getPinState() { return regs.gpio->getValue(); }

  // SET PIN POLARITY
  void setInputPolarity(PIN pin, INPUT_POLARITY pol) {
    assert(Util::getPortFromPin(pin) == port_name && "Invalid pin ");
    regs.ipol->setBitField(static_cast<uint8_t>(pin),
                           pol == INPUT_POLARITY::INVERTED);
  }
  void setInputPolarity(uint8_t pinmask, INPUT_POLARITY pol) {
    if (pol == INPUT_POLARITY::INVERTED) {
      regs.ipol->applyMask(pinmask);
    } else {
      regs.ipol->clearMask(pinmask);
    }
  }
  void setInputPolarity(INPUT_POLARITY pol) {
    if (pol == INPUT_POLARITY::INVERTED) {
      regs.ipol->applyMask(generalMask);
    } else {
      regs.ipol->clearMask(generalMask);
    }
  }

  // SET PIN VALUE
  void setPinState(PIN pin, bool state) {
    assert(Util::getPortFromPin(pin) == port_name && "Invalid pin ");
    regs.olat->setBitField(static_cast<uint8_t>(pin), state);
  }

  void setPinState(uint8_t pinmask, bool state) {
    if (state) {
      regs.olat->applyMask(pinmask);
    } else {
      regs.olat->clearMask(pinmask);
    }
  }

  void setPinState(bool state) {
    if (state) {
      regs.olat->applyMask(generalMask);
    } else {
      regs.olat->clearMask(generalMask);
    }
  }

private:
  MCP::MCP_MODEL model;
  bool bankMode = false;
  bool interruptEnabled = false;
  uint8_t generalMask;
  PORT port_name;

  void init() {}

  static constexpr std::array<Pin, PIN_PER_BANK> createPins(PORT port) {
    std::array<Pin, PIN_PER_BANK> pins{};
    for (size_t i = 0; i < PIN_PER_BANK; ++i) {
      pins[i] =
          Pin(static_cast<PIN>(static_cast<uint8_t>(port) * PIN_PER_BANK + i));
    }
    return pins;
  }
};

} // namespace MCP
#endif
