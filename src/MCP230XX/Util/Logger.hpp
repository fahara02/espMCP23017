#ifndef LOGGER_HPP
#define LOGGER_HPP
#include "Arduino.h"
#include <esp_log.h>
#include <stdarg.h>
#include <bitset>
#include <cstring>
#include "stdint.h"
static constexpr uint16_t LOG_BUFFER_SIZE = 256;
namespace LOG
{
enum class LogLevel
{
	INFO,
	DEBUG,
	WARNING,
	ERROR,
	SUCCESS,
	TEST
};

class Logger
{
  public:
	static Logger& getInstance()
	{
		static Logger instance;
		return instance;
	}

	void enable() { enabled_ = true; }

	void disable() { enabled_ = false; }

	void log(const char* tag, LogLevel level, const char* format, ...)
	{
		if(!enabled_)
			return;

		va_list args;
		va_start(args, format);
		vlog(tag, level, format, args);
		va_end(args);
	}

	void vlog(const char* tag, LogLevel level, const char* format, va_list args)
	{
		if(!enabled_)
			return;

		char buffer[LOG_BUFFER_SIZE];
		char binaryBuffer[40]; // 32-bit binary + prefix

		// Detect if format contains "%b"
		if(strstr(format, "%b") != nullptr)
		{
			uint32_t value = va_arg(args, uint32_t); // Extract integer argument
			snprintf(binaryBuffer, sizeof(binaryBuffer), "0b%s",
					 std::bitset<32>(value).to_string().c_str());

			// Replace %b with the binary string
			char newFormat[LOG_BUFFER_SIZE];
			snprintf(newFormat, sizeof(newFormat), "%s", format);
			char* bPos = strstr(newFormat, "%b");
			if(bPos)
			{
				strcpy(bPos, binaryBuffer); // Replace "%b" with binary value
			}

			snprintf(buffer, sizeof(buffer), newFormat);
		}
		else
		{
			vsnprintf(buffer, sizeof(buffer), format, args);
		}

		const char* prefix = formatLogLevel(level);

		switch(level)
		{
			case LogLevel::INFO:
			case LogLevel::SUCCESS:
			case LogLevel::TEST:
				ESP_LOGI(tag, "%s%s\033[0m", prefix, buffer);
				break;
			case LogLevel::DEBUG:
				ESP_LOGD(tag, "%s%s\033[0m", prefix, buffer);
				break;
			case LogLevel::WARNING:
				ESP_LOGW(tag, "%s%s\033[0m", prefix, buffer);
				break;
			case LogLevel::ERROR:
				ESP_LOGE(tag, "%s%s\033[0m", prefix, buffer);
				break;
		}
	}

  private:
	Logger() : enabled_(false), infoColorToggle_(false) {}

	Logger(const Logger&) = delete;
	Logger& operator=(const Logger&) = delete;

	bool enabled_;
	bool infoColorToggle_;

	const char* formatLogLevel(LogLevel level)
	{
		switch(level)
		{
			case LogLevel::INFO:
				infoColorToggle_ = !infoColorToggle_;
				return infoColorToggle_ ? "\033[37m[INFO] " : "\033[38;5;94m[INFO] ";
			case LogLevel::DEBUG:
				return "\033[34m[DEBUG] ";
			case LogLevel::WARNING:
				return "\033[33m[WARNING] ";
			case LogLevel::ERROR:
				return "\033[31m[ERROR] ";
			case LogLevel::SUCCESS:
				return "\033[94m[SUCCESS] ";
			case LogLevel::TEST:
				return "\033[36m[TEST] ";
			default:
				return "\033[0m";
		}
	}
};

// Wrapper functions with custom tag support
static void INFO(const char* tag, const char* format, ...)
{
	va_list args;
	va_start(args, format);
	Logger::getInstance().vlog(tag, LogLevel::INFO, format, args);
	va_end(args);
}

static void DEBUG(const char* tag, const char* format, ...)
{
	va_list args;
	va_start(args, format);
	Logger::getInstance().vlog(tag, LogLevel::DEBUG, format, args);
	va_end(args);
}

static void ERROR(const char* tag, const char* format, ...)
{
	va_list args;
	va_start(args, format);
	Logger::getInstance().vlog(tag, LogLevel::ERROR, format, args);
	va_end(args);
}

static void SUCCESS(const char* tag, const char* format, ...)
{
	va_list args;
	va_start(args, format);
	Logger::getInstance().vlog(tag, LogLevel::SUCCESS, format, args);
	va_end(args);
}

static void WARNING(const char* tag, const char* format, ...)
{
	va_list args;
	va_start(args, format);
	Logger::getInstance().vlog(tag, LogLevel::WARNING, format, args);
	va_end(args);
}

static void TEST(const char* tag, const char* format, ...)
{
	va_list args;
	va_start(args, format);
	Logger::getInstance().vlog(tag, LogLevel::TEST, format, args);
	va_end(args);
}

static void ENABLE() { Logger::getInstance().enable(); }

static void DISABLE() { Logger::getInstance().disable(); }

} // namespace LOG

#endif // LOGGER_HPP
