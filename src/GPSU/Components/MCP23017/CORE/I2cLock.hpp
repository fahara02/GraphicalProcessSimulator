#ifndef I2C_HPP
#define I2C_HPP
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class I2CLock {
public:
  I2CLock(SemaphoreHandle_t sem, TickType_t timeout)
      : sem_(sem), acquired_(false) {

    if (xSemaphoreTake(sem_, timeout) == pdTRUE) {
      acquired_ = true;
    } else {

      ESP_LOGE("I2CBUS", "Failed to take mutex");
    }
  }

  ~I2CLock() {

    if (acquired_) {
      xSemaphoreGive(sem_);
    }
  }

  bool acquired() const { return acquired_; }

  I2CLock(const I2CLock &) = delete;
  I2CLock &operator=(const I2CLock &) = delete;

private:
  SemaphoreHandle_t sem_;
  bool acquired_;
};

#endif