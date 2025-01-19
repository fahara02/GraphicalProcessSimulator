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

PulseCounter::PulseCounter(uint16_t max_count, uint16_t min_count,
                           bool interrupt_enable,
                           EncoderISRCallback isr_callback,
                           void *isr_callback_data, PullType pull_type)
    : isr_callback_(std::move(isr_callback)),
      isr_callback_data_(isr_callback_data), a_pin_(GPIO_NUM_NC),
      b_pin_(GPIO_NUM_NC), max_count_(max_count), min_count_(min_count),
      unit_((pcnt_unit_t)-1), pcnt_config_({}), pull_type_(pull_type),
      encoder_type_(EncoderType::FULL), counts_mode_(2), count_(0),
      interrupt_enabled_all_pulse_(interrupt_enable), attached_(false),
      direction_(false), working_(false) {}

PulseCounter::~PulseCounter() { detach(); }

static IRAM_ATTR void ipc_install_isr_on_core(void *arg) {
  esp_err_t *result = (esp_err_t *)arg;
  *result = pcnt_isr_service_install(0);
}

int64_t PulseCounter::incrementCount(int64_t delta) {
  return count_.fetch_add(delta, std::memory_order_seq_cst) + delta;
}

static void pcnt_intr_handler(void *arg) {

  PulseCounter *pc = static_cast<PulseCounter *>(arg);
  pcnt_config_t config = pc->getPCNTconfig();
  pcnt_unit_t unit = config.unit;
  int64_t new_count = 0;
  _ENTER_CRITICAL();
  // Handle high limit interrupt
  if (PCNT.status_unit[unit].COUNTER_H_LIM) {
    new_count = pc->incrementCount(config.counter_h_lim);
    PCNT.int_clr.val = (1 << unit); // Clear interrupt flag
    pcnt_counter_clear(unit);
  }
  // Handle low limit interrupt
  else if (PCNT.status_unit[unit].COUNTER_L_LIM) {
    new_count = pc->incrementCount(config.counter_l_lim);
    PCNT.int_clr.val = (1 << unit); // Clear interrupt flag
    pcnt_counter_clear(unit);

  }
  // Handle threshold interrupts

  else if (pc->isInterreptForAllPulseEnabled() &&
           (PCNT.status_unit[unit].thres0_lat ||
            PCNT.status_unit[unit].thres1_lat)) {
    int16_t raw_count = 0;
    pcnt_get_counter_value(unit, &raw_count);
    new_count = pc->incrementCount(raw_count);

    pcnt_set_event_value(unit, PCNT_EVT_THRES_0, -1);
    pcnt_set_event_value(unit, PCNT_EVT_THRES_1, 1);
    pcnt_event_enable(unit, PCNT_EVT_THRES_0);
    pcnt_event_enable(unit, PCNT_EVT_THRES_1);
    pcnt_counter_clear(unit);

    if (pc->isr_callback_) {
      pc->isr_callback_(pc->isr_callback_data_);
    }
  }

  _EXIT_CRITICAL();
}

void PulseCounter::configureGPIOs() {

  gpio_pad_select_gpio(a_pin_);
  gpio_pad_select_gpio(b_pin_);
  gpio_set_direction(a_pin_, GPIO_MODE_INPUT);
  gpio_set_direction(b_pin_, GPIO_MODE_INPUT);

  if (pull_type_ == PullType::INTERNAL_PULLUP) {
    gpio_pullup_en(a_pin_);
    gpio_pullup_en(b_pin_);
  } else if (pull_type_ == PullType::INTERNAL_PULLDOWN) {
    gpio_pulldown_en(a_pin_);
    gpio_pulldown_en(b_pin_);
  } else if (pull_type_ == PullType::NONE) {
    gpio_pullup_dis(a_pin_);
    gpio_pulldown_dis(a_pin_);
    gpio_pullup_dis(b_pin_);
    gpio_pulldown_dis(b_pin_);
  }
}

void PulseCounter::configurePCNT(EncoderType et) {
  // Channel 0 configuration
  pcnt_config_.pulse_gpio_num = a_pin_;
  pcnt_config_.ctrl_gpio_num = b_pin_;
  pcnt_config_.unit = unit_;
  pcnt_config_.counter_h_lim = max_count_;
  pcnt_config_.counter_l_lim = min_count_;

  if (et == EncoderType::SINGLE) {
    pcnt_config_.channel = PCNT_CHANNEL_0;
    pcnt_config_.pos_mode = PCNT_COUNT_DEC;      // Count Only On Rising-Edges
    pcnt_config_.neg_mode = PCNT_COUNT_INC;      // Discard Falling-Edge
    pcnt_config_.lctrl_mode = PCNT_MODE_KEEP;    // Rising A on HIGH B = CW Step
    pcnt_config_.hctrl_mode = PCNT_MODE_REVERSE; // Rising A on LOW B = CCW Step
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
    // diable first
    pcnt_config_.pos_mode = PCNT_COUNT_DIS;
    pcnt_config_.neg_mode = PCNT_COUNT_DIS;
    pcnt_config_.lctrl_mode = PCNT_MODE_DISABLE;
    pcnt_config_.hctrl_mode = PCNT_MODE_DISABLE;
    // then enable again
    pcnt_config_.pulse_gpio_num = b_pin_;
    pcnt_config_.ctrl_gpio_num = a_pin_;
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

void PulseCounter::attach(int a_pin, int b_pin, EncoderType encoder_type) {
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
  a_pin_ = static_cast<gpio_num_t>(a_pin);
  b_pin_ = static_cast<gpio_num_t>(b_pin);
  encoder_type_ = encoder_type;

  // Initialize GPIOs and PCNT
  configureGPIOs();
  configurePCNT(encoder_type);
  setFilter(250);
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
  if (interrupt_enabled_all_pulse_) {
    pcnt_set_event_value(unit_, PCNT_EVT_THRES_0, -1);
    pcnt_set_event_value(unit_, PCNT_EVT_THRES_1, 1);
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
  int16_t c;
  int64_t compensate = 0;
  _ENTER_CRITICAL();
  pcnt_get_counter_value(unit_, &c);

  if (PCNT.int_st.val & BIT(unit_)) {
    pcnt_get_counter_value(unit_, &c);
    if (PCNT.status_unit[unit_].COUNTER_H_LIM) {
      compensate = pcnt_config_.counter_h_lim;
    } else if (PCNT.status_unit[unit_].COUNTER_L_LIM) {
      compensate = pcnt_config_.counter_l_lim;
    }
  }
  _EXIT_CRITICAL();
  return compensate + c;
}

}; // namespace COMPONENT
