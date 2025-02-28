#include "test_mcp23017.hpp"
#include "Logger.hpp"
#include <unity.h>
#include "driver/gpio.h"
#define TAG "MCPTest"
gpio_num_t sda = GPIO_NUM_25;
gpio_num_t scl = GPIO_NUM_33;
gpio_num_t reset = GPIO_NUM_13;
MockMCP mcp;
void cb1(void* data) { Serial.println(" lower limit reached"); }
void cb2(void* data) { Serial.println(" Higher limit reached"); }

void cb3(int* data) { Serial.println(" lower limit reached"); }
void cb4(int* data) { Serial.println(" Higher limit reached"); }

void MockMCP::initialise()
{

	mcp.init();
	mcp.dumpRegisters();
}
void MockMCP::runAllTests()
{
	LOG::ENABLE();
	RUN_TEST(testMCPSetting);
	RUN_TEST(testsetInterrupts);
	RUN_TEST(testMCPPinMode);
	RUN_TEST(testMCPDigitalRead);
	RUN_TEST(testMCPDigitalWrite);
}

void MockMCP::testMCPSetting()
{
	MCP::Settings setting;
	setting.opMode = MCP::OperationMode::SequentialMode16;
	mcp.configure(setting);
	delay(1000);
	mcp.dumpRegisters();
	TEST_ASSERT_EQUAL(true, true);
}
void MockMCP::testsetInterrupts()
{
	int sensorThershold = 12;
	mcp.setInterrupts(GPB1, GPB2, RISING);
	mcp.setInterrupts(GPB3, GPB4, RISING, MCP::INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH);
	mcp.setInterrupts(GPB5, cb1, GPB6, cb2, RISING, MCP::INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH);
	mcp.setInterrupts(GPB7, cb3, &sensorThershold, GPB0, cb4, &sensorThershold, RISING,
					  MCP::INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH);
	mcp.dumpRegisters();
}
void MockMCP::testMCPPinMode()
{
	mcp.pinMode(OUTPUT_OPEN_DRAIN, GPA1, GPA2, GPA3, GPA4);
	delay(1000);
	mcp.pinMode(MCP::PORT::GPIOB, INPUT);
	delay(1000);
	mcp.invertInput(true, GPB1, GPB2, GPB3, GPB4);
	delay(1000);
	TEST_ASSERT_EQUAL(true, true);
}
void MockMCP::testMCPDigitalRead()
{
	mcp.digitalWrite(true, GPA1, GPA2, GPA3, GPA4);
	uint8_t result = mcp.digitalRead(GPA1, GPA2, GPA3, GPA4);
	TEST_ASSERT_EQUAL(30, result);
	mcp.digitalWrite(false, GPA1, GPA2, GPA3, GPA4);
	result = mcp.digitalRead(GPA1, GPA2, GPA3, GPA4);
	TEST_ASSERT_EQUAL(0, result);
}
void MockMCP::testMCPDigitalWrite()
{
	for(int i = 0; i < 50; i++)
	{
		mcp.digitalWrite(true, GPA1, GPA2, GPA3, GPA4);
		delay(1000);
		mcp.digitalWrite(false, GPA1, GPA2, GPA3, GPA4);
	}

	TEST_ASSERT_EQUAL(true, true);
}