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
      bg_(std::make_unique<TFT_eSprite>(canvas_.get())),
      img_(std::make_unique<TFT_eSprite>(canvas_.get())),
      frame_(std::make_unique<TFT_eSprite>(canvas_.get())),
      menu_(std::make_unique<MenuSelector>(menuItems, MENU_ITEM_COUNT, 4, pinA,
                                           pinB, btn)),
      current_process_(GPSU::ProcessType::ANY) {}
//------------------------------------------------------------------------------
// Accessor functions
//------------------------------------------------------------------------------
TFT_eSPI &Display::Canvas() { return *canvas_; }

TFT_eSprite &Display::Sprite() { return *img_; }
//------------------------------------------------------------------------------
// show_menu (//https://barth-dev.de/online/rgb565-color-picker/)
// https://www.iconarchive.com/show/farm-fresh-icons-by-fatcow/traffic-lights-green-icon.html
// http: // www.rinkydinkelectronics.com/
// https://oleddisplay.squix.ch/#/home
//------------------------------------------------------------------------------

void Display::showMenu() {
  // Clear the screen with a black background
  canvas_->fillScreen(TFT_BLACK);
  // Get the index of the selected menu item
  size_t selected_index = menu_->get_selected_index();

  // Define constants for layout
  const int16_t padding = PADDING_PX;
  const int16_t fontSize = MENU_FONT;
  const int16_t itemHeight =
      canvas_->fontHeight(fontSize) + MENU_VERTICAL_PADDING;

  // Loop through all menu items
  for (size_t i = 0; i < sizeof(menuItems) / sizeof(menuItems[0]); i++) {

    int16_t yPos = padding + (i * itemHeight);

    // Draw a background rectangle for the selected item
    if (i == selected_index) {
      canvas_->fillRect(0,                // x-start (left edge)
                        yPos,             // y-start
                        canvas_->width(), // width (full screen width)
                        itemHeight,       // height of the item
                        static_cast<uint16_t>(Colors::logo) // Highlight color
      );
    }

    // Calculate text dimensions
    int16_t textWidth = canvas_->textWidth(menuItems[i].label, fontSize);
    int16_t textHeight = canvas_->fontHeight(fontSize);

    // Calculate x-position to center the text horizontally
    int16_t xPos = (canvas_->width() - textWidth) / 2;

    // Adjust y-position to center the text vertically within the item height
    int16_t adjustedYPos = yPos + (itemHeight - textHeight) / 2;

    // Set text and background colors
    Colors textColor = Colors::main; // Foreground color for text
    Colors bgColor = (i == selected_index) ? Colors::logo
                                           : Colors::black; // Background color
    canvas_->setTextColor(static_cast<uint16_t>(textColor),
                          static_cast<uint16_t>(bgColor));

    // Draw the menu item's text at the calculated position
    canvas_->drawString(menuItems[i].label, xPos, adjustedYPos, fontSize);
  }
}

void Display::showSubMenu() {
  const int16_t fontSize = MENU_FONT;
  canvas_->fillScreen(TFT_BLACK);
  canvas_->fillRect(LEFT_MARGIN_PX,   // x-start (left edge)
                    TOP_MARGIN_PX,    // y-start
                    canvas_->width(), // width (full screen width)
                    140,              // height of the item
                    static_cast<uint16_t>(Colors::main) // Highlight color
  );
  size_t selected_index = menu_->get_selected_index();
  canvas_->drawString(menuItems[selected_index].label, 10, 70, fontSize);
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

  case AlignMent::CENTER_MIDDLE:
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
void Display::drawAlignText(AlignMent align, const char *text, uint8_t font,
                            Colors txt, Colors bg) {
  if (!text) {
    ESP_LOGE("GUI", "Null pointer passed!");
    return;
  }

  int16_t textWidth = canvas_->textWidth(text, font);
  int16_t textHeight = canvas_->fontHeight(font);

  auto [x, y] = calculateAlignment(align, textWidth, textHeight);
  canvas_->setTextColor(static_cast<uint16_t>(txt), static_cast<uint16_t>(bg));
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
  // show_menu();
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
  getInstance().showMenu();
}

//------------------------------------------------------------------------------
// Static callback for when a menu item is confirmed.
//------------------------------------------------------------------------------
void Display::onItemSelected(size_t index) {
  Serial.println("Item confirmed");
  getInstance().showSubMenu();
  menuItems[index].action();
}
} // namespace GUI