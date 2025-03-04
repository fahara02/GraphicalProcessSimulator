#include "GUI/GUI.hpp"

#include "Utility/gpsuUtility.hpp"

namespace GUI {
gpio_num_t pinA = GPIO_NUM_37;
gpio_num_t pinB = GPIO_NUM_38;
gpio_num_t btn = GPIO_NUM_32;
const MenuItem Display::menuItems[] = {
    {"TRAFFIC_LIGHT", Display::processTrafficLight},
    {"WATER_LEVEL", Display::processWaterLevel},
    {"STEPPER_MOTOR_CONTROL", Display::processStepperMotorControl},
    {"STATE_MACHINE", Display::processStateMachine},
    {"OBJECT_COUNTER", Display::processObjectCounter},
    {"MOTOR_CONTROl", Display::processMotorControl}};
Display &Display::getInstance() {
  static Display instance;
  return instance;
}
Display::Display()
    : initialised(false), canvas_(std::make_unique<TFT_eSPI>()),
      sprite_(std::make_unique<TFT_eSprite>(canvas_.get())),
      menu_(std::make_unique<MenuSelector>(menuItems, MENU_ITEM_COUNT, 4, pinA,
                                           pinB, btn)),
      current_process_(GPSU::ProcessType::ANY) {}
//------------------------------------------------------------------------------
// Accessor functions
//------------------------------------------------------------------------------
TFT_eSPI &Display::Canvas() { return *canvas_; }

TFT_eSprite &Display::Sprite() { return *sprite_; }
//------------------------------------------------------------------------------
// show_menu (//https://barth-dev.de/online/rgb565-color-picker/)
//------------------------------------------------------------------------------

void Display::show_menu() {
  canvas_->fillScreen(TFT_BLACK);
  drawAlignText(AlignMent::TOP_MIDDLE,
                GPSU::Util::ToString::Process(current_process_), 2);
}

std::tuple<int16_t, int16_t>
Display::calculateAlignment(AlignMent align, int16_t Width, int16_t Height) {
  int16_t x = 0;
  int16_t y = 0;

  switch (align) {
  case AlignMent::TOP_MIDDLE:
    x = (MAX_WIDTH - Width) / 2;
    y = TOP_MARGIN_PX;
    break;

  case AlignMent::MIDDLE:
    x = (MAX_WIDTH - Width) / 2;
    y = (MAX_HEIGHT - Height) / 2;
    break;

  case AlignMent::LEFT_X:
    x = LEFT_MARGIN_PX;
    y = (MAX_HEIGHT - Height) / 2;
    break;

  case AlignMent::RIGHT_X:
    x = MAX_WIDTH - Width - RIGHT_MARGIN_PX;
    y = (MAX_HEIGHT - Height) / 2;
    break;

  case AlignMent::BOTTOM_MIDDLE:
    x = (MAX_WIDTH - Width) / 2;
    y = MAX_HEIGHT - Height - BOTTOM_MARGIN_PX;
    break;

  default:
    ESP_LOGW("GUI", "Unknown alignment, defaulting to TOP_MIDDLE");
    x = (MAX_WIDTH - Width) / 2;
    y = TOP_MARGIN_PX;
    break;
  }

  return std::make_tuple(x, y);
}
void Display::drawAlignText(AlignMent align, const char *text, uint8_t font) {
  if (!text) {
    ESP_LOGE("GUI", "Null pointer passed!");
    return;
  }

  int16_t textWidth = canvas_->textWidth(text, font);
  int16_t textHeight = canvas_->fontHeight(font);

  auto [x, y] = calculateAlignment(align, textWidth, textHeight);

  canvas_->drawString(text, x, y, font);
}

//------------------------------------------------------------------------------
// Static callback functions for each menu item.
//------------------------------------------------------------------------------
void Display::processTrafficLight() {
  getInstance().run_process(GPSU::ProcessType::TRAFFIC_LIGHT);
}

void Display::processWaterLevel() {
  getInstance().run_process(GPSU::ProcessType::WATER_LEVEL);
}

void Display::processStepperMotorControl() {
  getInstance().run_process(GPSU::ProcessType::STEPPER_MOTOR_CONTROL);
}

void Display::processStateMachine() {
  getInstance().run_process(GPSU::ProcessType::STATE_MACHINE);
}

void Display::processObjectCounter() {
  getInstance().run_process(GPSU::ProcessType::OBJECT_COUNTER);
}

void Display::processMotorControl() {
  getInstance().run_process(GPSU::ProcessType::MOTOR_CONTROl);
}

//------------------------------------------------------------------------------
// Initialisation function.
//------------------------------------------------------------------------------
void Display::init() {
  if (!initialised) {
    menu_->set_selection_changed_Cb(onSelectionChanged);
    menu_->set_item_selected_cb(onItemSelected);
    menu_->init();
    canvas_->init();
    initialised = true;
  }
}

//------------------------------------------------------------------------------
// Runs a process based on the ProcessType.
//------------------------------------------------------------------------------
void Display::run_process(GPSU::ProcessType type) {
  current_process_ = type;
  show_menu();
  switch (type) {
  case GPSU::ProcessType::TRAFFIC_LIGHT:
    Serial.println("Running traffic light process");
    break;
  case GPSU::ProcessType::WATER_LEVEL:
    Serial.println("Running water level process");
    break;
  case GPSU::ProcessType::STEPPER_MOTOR_CONTROL:
    Serial.println("Running stepper motor process");
    break;
  case GPSU::ProcessType::STATE_MACHINE:
    Serial.println("Running state machine process");
    break;
  case GPSU::ProcessType::OBJECT_COUNTER:
    Serial.println("Running object counter process");
    break;
  case GPSU::ProcessType::MOTOR_CONTROl:
    Serial.println("Running motor control process");
    break;
  default:
    Serial.println("Unknown process");
    break;
  }
}

//------------------------------------------------------------------------------
// Static callback for when the menu selection changes.
//------------------------------------------------------------------------------
void Display::onSelectionChanged(size_t index) {
  Serial.print("Selected: ");
  Serial.println(menuItems[index].label);
}

//------------------------------------------------------------------------------
// Static callback for when a menu item is confirmed.
//------------------------------------------------------------------------------
void Display::onItemSelected(size_t index) { Serial.println("Item confirmed"); }
} // namespace GUI