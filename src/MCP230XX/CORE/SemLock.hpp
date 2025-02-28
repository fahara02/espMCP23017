#ifndef MUTEX_LOCK_HPP
#define MUTEX_LOCK_HPP
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class SemLock {
public:
  SemLock(SemaphoreHandle_t sem, TickType_t timeout)
      : sem_(sem), acquired_(false) {

    if (xSemaphoreTake(sem_, timeout) == pdTRUE) {
      acquired_ = true;
    } else {

      ESP_LOGE("SemLock", "Failed to take mutex");
    }
  }

  ~SemLock() {

    if (acquired_) {
      xSemaphoreGive(sem_);
    }
  }

  bool acquired() const { return acquired_; }

  SemLock(const SemLock &) = delete;
  SemLock &operator=(const SemLock &) = delete;

private:
  SemaphoreHandle_t sem_;
  bool acquired_;
};

#endif