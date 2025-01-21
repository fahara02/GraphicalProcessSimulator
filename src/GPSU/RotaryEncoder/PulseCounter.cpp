#include "PulseCounter.hpp"
#include "esp_ipc.h"
#include "esp_log.h"
#include <soc/soc_caps.h>

#define TAG_ENCODER "PulseCounter"

#ifdef CONFIG_IDF_TARGET_ESP32S2
#define COUNTER_H_LIM cnt_thr_h_lim_lat_un
#define COUNTER_L_LIM cnt_thr_l_lim_lat_un
#define thres0_lat cnt_thr_thres0_lat_un
#define thres1_lat cnt_thr_thres1_lat_un

#elif CONFIG_IDF_TARGET_ESP32S3
#define COUNTER_H_LIM cnt_thr_h_lim_lat_un
#define COUNTER_L_LIM cnt_thr_l_lim_lat_un
#define thres0_lat cnt_thr_thres0_lat_un
#define thres1_lat cnt_thr_thres1_lat_un
#else
#define COUNTER_H_LIM h_lim_lat
#define COUNTER_L_LIM l_lim_lat
#endif

namespace COMPONENT {

uint32_t PulseCounter::isr_service_cpu_core_ = 0;
bool PulseCounter::attached_interrupt_ = false;
std::array<std::unique_ptr<PulseCounter>, PulseCounter::MAX_ENCODERS>
    PulseCounter::encoders_ = {};

PulseCounter::PulseCounter(bool interrupt_enable,
                           EncoderISRCallback isr_callback,
                           void *isr_callback_data, PullType pull_type,
                           bool circular)
    : isr_callback_(std::move(isr_callback)),
      isr_callback_data_(isr_callback_data), sig_io_(GPIO_NUM_NC),
      cntrl_io_(GPIO_NUM_NC), unit_((pcnt_unit_t)-1), pcnt_config_({}),
      pull_type_(pull_type), encoder_type_(EncoderType::FULL), counts_mode_(2),
      count_(0), all_pulse_interrupt_enabled_(interrupt_enable),
      attached_(false), direction_(false), working_(false),
      circular_value_(circular) {
  init();
}

PulseCounter::~PulseCounter() { detach(); }

void PulseCounter::init() {
  if (!pcnt_evt_queue_) {
    pcnt_evt_queue_ = xQueueCreate(PCNT_EVT_QUEUE_SIZE, sizeof(pcnt_evt_t));
  }
  xTaskCreatePinnedToCore(
      pcntmonitorTask, "PCNTEventMonitor", GPSU_CORE::PCNTaskStack, this,
      GPSU_CORE::PCNTask_Priority, &pcnt_evt_task, GPSU_CORE::PCNTask_CORE);
}

static IRAM_ATTR void ipc_install_isr_on_core(void *arg) {
  esp_err_t *result = (esp_err_t *)arg;
  *result = pcnt_isr_service_install(0);
}

int64_t PulseCounter::incrementCount(int64_t delta) {
  return count_.fetch_add(delta, std::memory_order_seq_cst) + delta;
}
void PulseCounter::pcnt_intr_handler(void *arg) {
  unsigned long now = esp_timer_get_time();
  uint32_t intr_status = PCNT.int_st.val;
  portBASE_TYPE HPTaskAwoken = pdFALSE;

  for (int i = 0; i < PulseCounter::MAX_ENCODERS; ++i) {
    if (intr_status & BIT(i)) {
      auto &encoder = PulseCounter::encoders_[i];
      if (encoder) {
        pcnt_evt_t evt;
        evt.unit = i;
        evt.status = PCNT.status_unit[i].val;
        evt.timeStamp = now;
        PCNT.int_clr.val = BIT(i);
        xQueueSendFromISR(encoder->pcnt_evt_queue_, &evt, &HPTaskAwoken);
      }
    }
  }

  if (HPTaskAwoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void PulseCounter::pcntmonitorTask(void *param) {
  PulseCounter *pc = static_cast<PulseCounter *>(param);
  pcnt_config_t config = pc->getPCNTconfig();
  pcnt_range_t range = pc->gtRange();
  pcnt_unit_t unit = config.unit;

  pcnt_evt_t evt;
  portBASE_TYPE res;
  while (true) {
    res = xQueueReceive(pc->pcnt_evt_queue_, &evt, pdMS_TO_TICKS(1000));
    auto &encoder = PulseCounter::encoders_[evt.unit];
    if (!encoder)
      continue;
    int64_t new_count = 0;
    if (res == pdTRUE) {

      // Handle high limit interrupt
      if (evt.status & PCNT_EVT_H_LIM) {
        new_count = pc->incrementCount(range.maxLimit);
        new_count = pc->wrapCountIfCircular(new_count);
        pcnt_counter_clear(unit);
      }
      // Handle low limit interrupt
      else if (evt.status & PCNT_EVT_L_LIM) {
        new_count = pc->incrementCount(range.minLimit);
        new_count = pc->wrapCountIfCircular(new_count);
        pcnt_counter_clear(unit);

      }
      // Handle threshold interrupts

      else if (pc->isInterreptForAllPulseEnabled() &&
               ((evt.status & PCNT_EVT_THRES_0) ||
                (evt.status & PCNT_EVT_THRES_1))) {
        int16_t raw_count = 0;
        pcnt_get_counter_value(unit, &raw_count);
        new_count = pc->incrementCount(raw_count);
        new_count = pc->wrapCountIfCircular(new_count);

        pcnt_set_event_value(unit, PCNT_EVT_THRES_0, range.thresh0);
        pcnt_set_event_value(unit, PCNT_EVT_THRES_1, range.thresh1);
        pcnt_event_enable(unit, PCNT_EVT_THRES_0);
        pcnt_event_enable(unit, PCNT_EVT_THRES_1);
        pcnt_counter_clear(unit);

        if (pc->isr_callback_) {
          pc->isr_callback_(pc->isr_callback_data_);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  vTaskDelete(NULL);
}

void PulseCounter::configureGPIOs() {
  ESP_LOGI(TAG_ENCODER, "Configuring GPIOs...");

  // Create gpio_config_t structure to configure pins
  gpio_config_t io_conf = {};

  // Set pin configuration for pin A
  io_conf.pin_bit_mask = (1ULL << sig_io_);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_down_en = (pull_type_ == PullType::INTERNAL_PULLDOWN)
                             ? GPIO_PULLDOWN_ENABLE
                             : GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = (pull_type_ == PullType::INTERNAL_PULLUP)
                           ? GPIO_PULLUP_ENABLE
                           : GPIO_PULLUP_DISABLE;
  io_conf.intr_type =
      GPIO_INTR_DISABLE; // No interrupt configuration in this example
  gpio_config(&io_conf);

  // Set pin configuration for pin B (similar to pin A)
  io_conf.pin_bit_mask = (1ULL << cntrl_io_);
  gpio_config(&io_conf);
}

void PulseCounter::configurePCNT(EncoderType et, pcnt_range_t range) {
  ESP_LOGI(TAG_ENCODER, "configuring PCNT");

  // Channel 0 configuration
  pcnt_config_.pulse_gpio_num = sig_io_;
  pcnt_config_.ctrl_gpio_num = cntrl_io_;
  pcnt_config_.unit = unit_;
  pcnt_config_.counter_h_lim = range.maxLimit;
  pcnt_config_.counter_l_lim = range.minLimit;

  if (et == EncoderType::SINGLE) {
    pcnt_config_.channel = PCNT_CHANNEL_0;
    pcnt_config_.pos_mode = PCNT_COUNT_INC;   // Count Only On Rising-Edges
    pcnt_config_.neg_mode = PCNT_COUNT_INC;   // Discard Falling-Edge
    pcnt_config_.lctrl_mode = PCNT_MODE_KEEP; // Rising A on HIGH B = CW Step
    pcnt_config_.hctrl_mode = PCNT_MODE_KEEP; // Rising A on LOW B = CCW Step
    pcnt_unit_config(&pcnt_config_);
  } else if (et == EncoderType::HALF) {
    pcnt_config_.channel = PCNT_CHANNEL_0;
    pcnt_config_.pos_mode = PCNT_COUNT_DIS;
    pcnt_config_.neg_mode = PCNT_COUNT_INC;
    pcnt_config_.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_unit_config(&pcnt_config_);
  } else if (et == EncoderType::FULL) {
    pcnt_config_.channel = PCNT_CHANNEL_0;
    pcnt_config_.pos_mode = PCNT_COUNT_DEC;
    pcnt_config_.neg_mode = PCNT_COUNT_INC;
    pcnt_config_.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_unit_config(&pcnt_config_);
    // then enable again
    pcnt_config_.pulse_gpio_num = cntrl_io_;
    pcnt_config_.ctrl_gpio_num = sig_io_;
    pcnt_config_.channel = PCNT_CHANNEL_1;
    pcnt_config_.pos_mode = PCNT_COUNT_DEC;
    pcnt_config_.neg_mode = PCNT_COUNT_INC;
    pcnt_config_.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_unit_config(&pcnt_config_);
  }
}
void PulseCounter::detach() {
  pcnt_counter_pause(unit_);
  pcnt_isr_handler_remove(this->pcnt_config_.unit);
  PulseCounter::encoders_[unit_] = NULL;
  attached_ = false;
}
void PulseCounter::setFilter(uint16_t value) {
  if (value > 1023)
    value = 1023;
  if (value == 0) {
    pcnt_filter_disable(unit_);
  } else {
    pcnt_set_filter_value(unit_, value);
    pcnt_filter_enable(unit_);
  }
}

void PulseCounter::attach(int a_pin, int b_pin, pcnt_range_t range,
                          EncoderType encoder_type) {
  if (attached_) {
    ESP_LOGE(TAG_ENCODER, "Already attached!");
    return;
  }

  // Assign a free PCNT unit
  int index = 0;
  for (; index < MAX_ENCODERS; ++index) {
    if (encoders_[index] == nullptr) {
      encoders_[index] = std::unique_ptr<PulseCounter>(this);
      break;
    }
  }
  if (index == MAX_ENCODERS) {
    ESP_LOGE(TAG_ENCODER, "Too many encoders! Attach failed.");
    return;
  }

  unit_ = static_cast<pcnt_unit_t>(index);
  pcnt_range_ = range;
  sig_io_ = static_cast<gpio_num_t>(a_pin);
  cntrl_io_ = static_cast<gpio_num_t>(b_pin);
  encoder_type_ = encoder_type;

  // Initialize GPIOs and PCNT
  configureGPIOs();
  configurePCNT(encoder_type, range);
  setFilter(200);
  pcnt_event_enable(unit_, PCNT_EVT_H_LIM);
  pcnt_event_enable(unit_, PCNT_EVT_L_LIM);
  pcnt_counter_pause(unit_);

  // Enable interrupts if requested
  if (!attached_interrupt_) {
    attached_interrupt_ = installInterruptService();
  }

  if (pcnt_isr_handler_add(unit_, pcnt_intr_handler, this) != ESP_OK) {
    ESP_LOGE(TAG_ENCODER,
             "Encoder install interrupt handler for unit %d failed", unit_);
  }
  if (all_pulse_interrupt_enabled_) {
    pcnt_set_event_value(unit_, PCNT_EVT_THRES_0, range.thresh0);
    pcnt_set_event_value(unit_, PCNT_EVT_THRES_1, range.thresh1);
    pcnt_event_enable(unit_, PCNT_EVT_THRES_0);
    pcnt_event_enable(unit_, PCNT_EVT_THRES_1);
  }
  pcnt_counter_clear(unit_);
  pcnt_intr_enable(unit_);
  pcnt_counter_resume(unit_);
  attached_ = true;
}
bool PulseCounter::installInterruptService() {
  bool isr_installed = false;
#ifdef CONFIG_IDF_TARGET_ESP32S2 // esp32-s2 is single core, no ipc call
  esp_err_t er = pcnt_isr_service_install(0);
  if (er != ESP_OK) {
    ESP_LOGE(TAG_ENCODER, "Encoder install isr service failed");
  }

#else
  if (isr_service_cpu_core_ == ISR_CORE_USE_DEFAULT ||
      isr_service_cpu_core_ == xPortGetCoreID()) {
    esp_err_t er = pcnt_isr_service_install(0);
    isr_installed = er == ESP_OK ? true : false;
    if (er != ESP_OK) {
      ESP_LOGE(TAG_ENCODER, "Encoder install isr service on same core failed");
    }
  } else {
    esp_err_t ipc_ret_code = ESP_FAIL;
    esp_err_t er = esp_ipc_call_blocking(
        isr_service_cpu_core_, ipc_install_isr_on_core, &ipc_ret_code);
    isr_installed = er == ESP_OK ? true : false;
    if (er != ESP_OK) {
      ESP_LOGE(TAG_ENCODER,
               "IPC call to install isr service on core %ud failed",
               isr_service_cpu_core_);
    }
    if (ipc_ret_code != ESP_OK) {
      ESP_LOGE(TAG_ENCODER, "Encoder install isr service on core %ud failed",
               isr_service_cpu_core_);
    }
  }
#endif
  return isr_installed;
}

void PulseCounter::setCount(int64_t value) {
  _ENTER_CRITICAL();
  count_ = value - getRawCount();
  _EXIT_CRITICAL();
}
int64_t PulseCounter::getCount() const {
  _ENTER_CRITICAL();
  int64_t result = count_ + getRawCount();
  result = wrapCountIfCircular(result);
  _EXIT_CRITICAL();

  return result;
}
int64_t PulseCounter::clearCount() {
  _ENTER_CRITICAL();
  count_ = 0;
  _EXIT_CRITICAL();
  return pcnt_counter_clear(unit_);
}

int64_t PulseCounter::pauseCount() { return pcnt_counter_pause(unit_); }
int64_t PulseCounter::resumeCount() { return pcnt_counter_resume(unit_); }

int64_t PulseCounter::getRawCount() const {
  int16_t raw_count = 0;
  int64_t compensate = 0;
  auto range = gtRange();
  _ENTER_CRITICAL();
  pcnt_get_counter_value(unit_, &raw_count);

  if (PCNT.int_st.val & BIT(unit_)) {
    pcnt_get_counter_value(unit_, &raw_count);
    if (PCNT.status_unit[unit_].COUNTER_H_LIM) {
      compensate = range.maxLimit;
    } else if (PCNT.status_unit[unit_].COUNTER_L_LIM) {
      compensate = range.minLimit;
    }
  }
  _EXIT_CRITICAL();

  return compensate + raw_count;
}
int64_t PulseCounter::wrapCountIfCircular(int64_t count) const {
  if (circular_value_) {
    auto range = gtRange();
    if (count >= range.maxLimit) {
      count -= range.maxLimit;
    } else if (count < 0) {
      count += range.maxLimit;
    }
  }
  return count;
}

}; // namespace COMPONENT
