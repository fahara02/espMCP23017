#include <Arduino.h>
#include "MCP23017.hpp"
#include "Logger.hpp"
TaskHandle_t runTaskhandle = nullptr;
void RunTask(void* param);

static int send_req = 0;

gpio_num_t pinA = GPIO_NUM_37; // GPIO pin for Channel A
gpio_num_t pinB = GPIO_NUM_38;
gpio_num_t sda = GPIO_NUM_25;
gpio_num_t scl = GPIO_NUM_33;
gpio_num_t reset = GPIO_NUM_13;
COMPONENT::MCP23017 expander(sda, scl, reset);

void cb1(void* data) { Serial.println(" lower limit reached"); }
void cb2(void* data) { Serial.println(" Higher limit reached"); }

void cb3(int* data) { Serial.println(" lower limit reached"); }
void cb4(int* data) { Serial.println(" Higher limit reached"); }
void setup()
{

	Serial.begin(115200);

	delay(1000);
	LOG::ENABLE();
	// delay(1000);
	// // expander.cntrlRegA->separateBanks<MCP::REG::IOCON>();
	// delay(2000);
	expander.init();
	expander.dumpRegisters();
	delay(1000);
	MCP::Settings setting;
	setting.opMode = MCP::OperationMode::SequentialMode16;
	expander.configure(setting);
	delay(1000);

	// expander.enableInterrupt();

	expander.pinMode(OUTPUT_OPEN_DRAIN, GPA1, GPA2, GPA3, GPA4);
	delay(1000);
	expander.pinMode(MCP::PORT::GPIOB, INPUT);
	delay(1000);
	expander.invertInput(true, GPB1, GPB2, GPB3, GPB4);
	delay(1000);
	int sensorThershold = 12;
	expander.setInterrupts(GPB1, GPB2, RISING);
	expander.setInterrupts(GPB3, GPB4, RISING, MCP::INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH);
	expander.setInterrupts(GPB5, cb1, GPB6, cb2, RISING, MCP::INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH);
	expander.setInterrupts(GPB7, cb3, &sensorThershold, GPB0, cb4, &sensorThershold, RISING,
						   MCP::INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH);
	expander.dumpRegisters();

	xTaskCreatePinnedToCore(RunTask, "RunTask", 4196, &expander, 2, &runTaskhandle, 0);
}
void loop()
{

	Serial.println("....");
	delay(2000);
}
void RunTask(void* param)
{
	MCP::MCPDevice* expander = static_cast<MCP::MCPDevice*>(param);
	while(true)
	{
		Serial.println("....MAIN TASK......");
		// uint8_t readmask = 0;
		// readmask = expander->digitalRead(GPB1, GPB2, GPB3, GPB4);

		// vTaskDelay(pdMS_TO_TICKS(10));

		// expander->digitalWrite(MCP::PORT::GPIOA, readmask, true);
		// vTaskDelay(pdMS_TO_TICKS(10));

		expander->digitalWrite(true, GPA1, GPA2, GPA3, GPA4);

		send_req += 1;

		// expander->gpioBankA->setPinState(mask, false);
		vTaskDelay(pdMS_TO_TICKS(100));
		uint8_t readmask = expander->digitalRead(GPA1, GPA2, GPA3, GPA4);
		Serial.printf("pins value is %d", readmask);
		vTaskDelay(pdMS_TO_TICKS(10));
		send_req += 1;
		if(send_req == 2)
		{
			send_req = 0;
			expander->digitalWrite(false, GPA1, GPA2, GPA3, GPA4);
			vTaskDelay(pdMS_TO_TICKS(10));
			expander->digitalRead(GPB1, GPB2, GPB3, GPB4);
			Serial.printf("B ports value is %d", readmask);
			vTaskDelay(pdMS_TO_TICKS(10));
		}
		readmask = expander->digitalRead(GPA1, GPA2, GPA3, GPA4);
		vTaskDelay(pdMS_TO_TICKS(10));
		Serial.printf("pins value is %d", readmask);

		vTaskDelay(pdMS_TO_TICKS(10));

		Serial.printf("Total send request=%d\n", send_req);
		Serial.println("..........");
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	vTaskDelete(NULL);
}