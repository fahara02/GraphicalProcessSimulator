#include "GUI/GUI.hpp"

#include "Utility/gpsuUtility.hpp"
#include "esp_log.h"

namespace GUI {

Display &Display::getInstance(const MenuItem *items, size_t count) {
  static Display instance(items, count);
  return instance;
}
Display::Display(const MenuItem *items, size_t count)
    : menuItems_(items), menuItemCount_(count), initialised(false),
      canvas_(std::make_unique<TFT_eSPI>()),
      bg_(std::make_unique<TFT_eSprite>(canvas_.get())),
      label_(std::make_unique<TFT_eSprite>(canvas_.get())),
      analog_(std::make_unique<TFT_eSprite>(canvas_.get())),
      img_(std::make_unique<TFT_eSprite>(canvas_.get())),
      imgRotor_(std::make_unique<TFT_eSprite>(canvas_.get())),
      frame_(std::make_unique<TFT_eSprite>(canvas_.get())),
      current_process_(GPSU::ProcessType::ANY) {}

//------------------------------------------------------------------------------
// Initialisation function.
//------------------------------------------------------------------------------

void Display::init() {
  if (!initialised) {

    canvas_->init();
    bg_->createSprite(MAX_WIDTH, MAX_HEIGHT);
    label_->createSprite(MAX_WIDTH - TOP_MARGIN_PX,
                         MAX_HEIGHT - BOTTOM_MARGIN_PX);
    displayQueue = xQueueCreate(10, sizeof(Command));
    xTaskCreate(&Display::display_loop, "display_loop", 2048, this, 1, NULL);
    initialised = true;
  }
}
void Display::setCursorIndex(size_t index) { cursorIndex_ = index; }

void Display::deinitProcessFlags() {
  setup_traffic = false;
  setup_waterlevel = false;
  setup_stepper = false;
}
void Display::deleteSprites() {
  // except bg and label_
  if (img_->created()) {
    img_->deleteSprite();
  }
  if (imgRotor_->created()) {
    imgRotor_->deleteSprite();
  }
  if (frame_->created()) {
    frame_->deleteSprite();
  }
}
void Display::createSprites() {
  if (setup_traffic) {
    img_->createSprite(IMG_WIDTH, IMG_HEIGHT);
    img_->setSwapBytes(true);
  }
  if (setup_waterlevel) {
    frame_->createSprite(FRMAE_WIDTH, FRAME_HEIGHT);
  }
  if (setup_stepper) {
    img_->createSprite(IMG2_WIDTH, IMG2_HEIGHT);
    img_->setSwapBytes(true);
    imgRotor_->createSprite(IMG2_WIDTH, IMG2_HEIGHT);
    imgRotor_->setSwapBytes(true);
  }
}
void Display::sendDisplayCommand(const Command &cmd) {
  xQueueSend(displayQueue, &cmd, portMAX_DELAY);
}
void Display::display_loop(void *param) {
  Display *self = static_cast<Display *>(param);
  Command cmd;
  while (true) {
    if (xQueueReceive(self->displayQueue, &cmd, portMAX_DELAY) == pdPASS) {
      switch (cmd.type) {
      case CommandType::SHOW_MENU:
        self->setCursorIndex(cmd.cursor.index);
        self->showMenu();
        break;
      case CommandType::SHOW_PROCESS_SCREEN:
        self->current_process_ = cmd.process_type;
        self->showProcessScreen(cmd.process_type);
        break;
      case CommandType::UPDATE_TRAFFIC_LIGHT:
        if (self->current_process_ == GPSU::ProcessType::TRAFFIC_LIGHT) {
          self->updateTrafficLight(cmd);
        }
        break;
      case CommandType::UPDATE_WATER_LEVEL:
        if (self->current_process_ == GPSU::ProcessType::WATER_LEVEL) {
          // self->updateWaterLevelDisplay(cmd.water_level_state,
          // cmd.analog_ch0);
        }
        break;
      case CommandType::UPDATE_STEPPER:
        if (self->current_process_ == GPSU::ProcessType::STEPPER_MOTOR) {
          // self->updateStepperDisplay(cmd.analog_ch0, cmd.analog_ch1);
        }
        break;
      }
    }
  }
}
void Display::showProcessScreen(GPSU::ProcessType type) {
  bg_->fillSprite(static_cast<uint16_t>(Colors::logo));
  const int16_t fontSize = MENU_FONT;
  const char *label = GPSU::Util::ToString::Process(type);
  label_->setTextColor(static_cast<uint16_t>(Colors::main),
                       static_cast<uint16_t>(Colors::black));
  processScreenSetup();
  label_->fillSprite(static_cast<uint16_t>(Colors::black));
  label_->drawString(label, 0, 0, fontSize);
  label_->pushToSprite(bg_.get(), LABEL_LEFT_PX, LABEL_TOP_PX,
                       static_cast<uint16_t>(Colors::black));
  img_->pushToSprite(bg_.get(), 0, IMAGE_TOP_PX,
                     static_cast<uint16_t>(Colors::black));
  frame_->pushToSprite(bg_.get(), 0, 0, static_cast<uint16_t>(Colors::black));
  bg_->pushSprite(0, 0);
}

void Display::processScreenSetup() {
  bg_->fillSprite(TFT_BLACK);
  bg_->fillSprite(static_cast<uint16_t>(Colors::logo));
  img_->fillSprite(TFT_BLACK);
  if (setup_stepper) {
    imgRotor_->fillSprite(TFT_BLACK);
  }
  if (setup_waterlevel) {
    frame_->fillSprite(TFT_BLACK);
  }
  label_->fillSprite(TFT_BLACK);
  label_->setTextColor(static_cast<uint16_t>(Colors::white),
                       static_cast<uint16_t>(Colors::black));
}
void Display::processScreenExecute(int angle) {

  img_->pushToSprite(bg_.get(), 0, IMAGE_TOP_PX,
                     static_cast<uint16_t>(Colors::black));
  if (setup_stepper) {
    canvas_->setPivot(IMG2_WIDTH / 2, IMG2_HEIGHT / 2);
    imgRotor_->pushRotated(bg_.get(), angle,
                           static_cast<uint16_t>(Colors::black));
  }
  if (setup_waterlevel) {
    frame_->pushToSprite(bg_.get(), 5, FRAME_TOP_PX,
                         static_cast<uint16_t>(Colors::black));
  }
  label_->pushToSprite(bg_.get(), LABEL_LEFT_PX, LABEL_TOP_PX,
                       static_cast<uint16_t>(Colors::black));
  bg_->pushSprite(0, 0);
}

void Display::updateTrafficLight(Command cmd) {
  processScreenSetup();
  label_->drawString("TRAFFIC_LIGHT", 5, 5, MENU_FONT);
  TrafficLight::State state = cmd.states.tl_state;
  switch (state) {
  case TrafficLight::State::INIT: // Init
    img_->pushImage(0, 0, IMG_WIDTH, IMG_HEIGHT, Asset::blank_traffic);
    break;
  case TrafficLight::State::RED_STATE: // Red
    img_->pushImage(0, 0, IMG_WIDTH, IMG_HEIGHT, Asset::traffic_red);
    break;
  case TrafficLight::State::YELLOW_STATE: // Yellow
    img_->pushImage(0, 0, IMG_WIDTH, IMG_HEIGHT, Asset::traffic_yellow);
    break;
  case TrafficLight::State::GREEN_STATE: // Green
    img_->pushImage(0, 0, IMG_WIDTH, IMG_HEIGHT, Asset::traffic_green);
    break;
    // default:
    //   img_->pushImage(0, 0, IMG_WIDTH, IMG_HEIGHT, Asset::blank_traffic);
    //   break;
  }
  processScreenExecute();
}
// void Display::updateWaterLevelDisplay(const int state, int level) {
//   processScreenSetup();
//   // Draw labels
//   label_->drawString("Water-Level", 0, 0, MENU_FONT);

//   uint16_t waterColor = TFT_BLUE;
//   const int tankX = 0; // Centered within FRAME_WIDTH
//   const int tankY = 0;
//   // Calculate water height
//   int waterPixelHeight = (level * TANK_HEIGHT) / 100;
//   int waterY = tankY + TANK_BORDER_THK + (TANK_HEIGHT - waterPixelHeight);

//   switch (state) {
//   case 0:
//     waterColor = TFT_BLUE;
//     break;
//   case 1:
//     waterColor = TFT_CYAN;
//     break;
//   case 2:
//     waterColor = TFT_GREEN;
//     break;
//   case 3:
//     waterColor = TFT_RED;
//     break;
//   default:
//     waterColor = TFT_YELLOW;
//     break;
//   }

//   label_->drawString(GPSU::Util::ToString::TankState(state), 0, 20,
//   MENU_FONT); uint16_t litre = (tank_capacity_litre * level) / 100;
//   label_->setTextColor(TFT_WHITE);
//   label_->drawString(String(litre), 0, 120, 7);

//   // Draw thick tank border
//   frame_->fillRoundRect(tankX, tankY, FRMAE_WIDTH, FRAME_HEIGHT, TANK_RADIUS,
//                         TFT_WHITE);
//   frame_->fillRoundRect(tankX + TANK_BORDER_THK, tankY + TANK_BORDER_THK,
//                         TANK_WIDTH, TANK_HEIGHT, TANK_RADIUS, TFT_BLACK);
//   // Draw water
//   frame_->fillRect(tankX + TANK_BORDER_THK, waterY, TANK_WIDTH,
//                    waterPixelHeight, waterColor);

//   processScreenExecute();
// }
void Display::updateStepperDisplay(const int dir, int step) {
  processScreenSetup();

  label_->drawString("Stepper-Motor", 0, 0, MENU_FONT);
  img_->pushImage(0, 0, IMG2_WIDTH, IMG2_HEIGHT,
                  Asset::stepper); // 130X130

  int stator_length = 80;
  int stator_width = 25;
  int stator_x = (IMG2_WIDTH - stator_length) / 2;
  int stator_y = (IMG2_HEIGHT - stator_width) / 2;

  imgRotor_->fillRoundRect(stator_x, stator_y, stator_length, stator_width, 20,
                           TFT_BLUE);

  int angle = 0;
  if (dir == 1) {
    angle = 45 + step * 1;

  } else if (dir == -1) {
    angle = 45 - step * 1;
  }
  if (angle > 360) {
    angle = 0;
  }
  label_->drawString(String(angle), 0, 20, 4);
  processScreenExecute(angle);
}

//------------------------------------------------------------------------------
// Accessor functions
//------------------------------------------------------------------------------
TFT_eSPI &Display::Canvas() { return *canvas_; }

TFT_eSprite &Display::Sprite() { return *img_; }
//------------------------------------------------------------------------------
//  (//https://barth-dev.de/online/rgb565-color-picker/)
// www.iconarchive.com/show/farm-fresh-icons-by-fatcow/traffic-lights-green-icon.html
// www.cleanpng.com/png-traffic-light-indicating-different-types-of-traffi-7915271/download-png.html
// https://www.pngwing.com/en/free-png-yjwqm/download
// http: // www.rinkydinkelectronics.com/
// https://oleddisplay.squix.ch/#/home
//------------------------------------------------------------------------------

// set menuItems

void Display::setMenuItems(const MenuItem *items, size_t count) {
  menuItems_ = items;
  menuItemCount_ = count;
}

// StartUp menu of GUI
void Display::showMenu() {

  //   size_t selected_index = cursorIndex_;
  //   size_t num_items = menuItemCount_;
  current_process_ = GPSU::ProcessType::ANY;
  // Clear the screen with a black background
  bg_->fillScreen(TFT_BLACK);

  // Define constants for layout
  const int16_t padding = PADDING_PX;
  const int16_t fontSize = MENU_FONT;
  const int16_t textHeight = bg_->fontHeight(fontSize);
  const int16_t itemHeight = textHeight + MENU_VERTICAL_PADDING;
  // Define frame padding
  const int16_t frame_padding = 10; // Adjustable padding around the frame
  // Calculate frame top and bottom positions
  int16_t y_frame_top = padding - frame_padding;
  if (y_frame_top < 0)
    y_frame_top = 0; // Ensure it doesn’t go off-screen
  int16_t y_frame_bottom =
      padding + (menuItemCount_ + 1) * itemHeight + frame_padding;
  if (y_frame_bottom > bg_->height())
    y_frame_bottom = bg_->height(); // Cap at screen height

  // Draw the frame around the menu
  bg_->drawRect(0, y_frame_top, bg_->width(), y_frame_bottom - y_frame_top,
                static_cast<uint16_t>(Colors::white));

  // Draw the title "Main Menu"
  const char *title = "Main Menu";
  int16_t title_width = bg_->textWidth(title, fontSize);
  int16_t x_title = (bg_->width() - title_width) / 2; // Center horizontally
  int16_t y_title =
      padding + (itemHeight - textHeight) / 2; // Center vertically in slot
  bg_->setTextColor(static_cast<uint16_t>(Colors::main),
                    static_cast<uint16_t>(Colors::black));
  bg_->drawString(title, x_title, y_title, fontSize);
  // Loop through all menu items, shifted down by one itemHeight
  for (size_t i = 0; i < menuItemCount_; i++) {
    int16_t yPos = padding + (i + 1) * itemHeight; // Shifted position

    // Draw a background rectangle for the selected item
    if (i == cursorIndex_) {
      bg_->fillRect(0, yPos, bg_->width(), itemHeight,
                    static_cast<uint16_t>(Colors::logo));
    }

    // Calculate text dimensions
    int16_t textWidth = bg_->textWidth(menuItems_[i].label, fontSize);
    // Calculate x-position to center the text horizontally
    int16_t xPos = (bg_->width() - textWidth) / 2;
    // Adjust y-position to center the text vertically within the item
    // height
    int16_t adjustedYPos = yPos + (itemHeight - textHeight) / 2;
    // Set text and background colors
    Colors textColor = Colors::main;
    Colors bgColor = (i == cursorIndex_) ? Colors::logo : Colors::black;
    bg_->setTextColor(static_cast<uint16_t>(textColor),
                      static_cast<uint16_t>(bgColor));

    // Draw the menu item's text
    bg_->drawString(menuItems_[i].label, xPos, adjustedYPos, fontSize);
  }

  // Push the sprite to the display
  bg_->pushSprite(0, 0);
}

//------------------------------------------------------------------------------
// Runs a process based on the ProcessType.
//------------------------------------------------------------------------------
void Display::run_process(GPSU::ProcessType type) {
  current_process_ = type;
  // show_menu();
  switch (type) {
  case GPSU::ProcessType::TRAFFIC_LIGHT:
    deinitProcessFlags();
    setup_traffic = true;
    deleteSprites();
    createSprites();

    Serial.println("Running traffic light process");
    break;
  case GPSU::ProcessType::WATER_LEVEL:
    deinitProcessFlags();
    setup_waterlevel = true;
    deleteSprites();
    createSprites();

    Serial.println("Running water level process");
    break;
  case GPSU::ProcessType::STEPPER_MOTOR:
    deinitProcessFlags();
    setup_stepper = true;
    deleteSprites();
    createSprites();

    Serial.println("Running stepper motor process");
    break;
  case GPSU::ProcessType::STATE_MACHINE:
    Serial.println("Running state machine process");
    break;
  case GPSU::ProcessType::OBJECT_COUNTER:
    Serial.println("Running object counter process");
    break;
  case GPSU::ProcessType::MOTOR_CONTROL:
    Serial.println("Running motor control process");
    break;
  default:
    Serial.println("Unknown process");
    break;
  }
}

} // namespace GUI

// // std::tuple<int16_t, int16_t>
// //  Display::calculateAlignment(AlignMent align, int16_t Width, int16_t
// //  Height) {
// //    int16_t x = 0;
// //    int16_t y = 0;

// //   switch (align) {
// //   case AlignMent::TOP_MIDDLE:
// //     x = (MAX_WIDTH - Width) / 2;
// //     y = TOP_MARGIN_PX;
// //     break;

// //   case AlignMent::CENTER_MIDDLE:
// //     x = (MAX_WIDTH - Width) / 2;
// //     y = (MAX_HEIGHT - Height) / 2;
// //     break;

// //   case AlignMent::LEFT_X:
// //     x = LEFT_MARGIN_PX;
// //     y = (MAX_HEIGHT - Height) / 2;
// //     break;

// //   case AlignMent::RIGHT_X:
// //     x = MAX_WIDTH - Width - RIGHT_MARGIN_PX;
// //     y = (MAX_HEIGHT - Height) / 2;
// //     break;

// //   case AlignMent::BOTTOM_MIDDLE:
// //     x = (MAX_WIDTH - Width) / 2;
// //     y = MAX_HEIGHT - Height - BOTTOM_MARGIN_PX;
// //     break;

// //   default:
// //     ESP_LOGW("GUI", "Unknown alignment, defaulting to TOP_MIDDLE");
// //     x = (MAX_WIDTH - Width) / 2;
// //     y = TOP_MARGIN_PX;
// //     break;
// //   }

// //   return std::make_tuple(x, y);
// // }
// // void Display::drawAlignText(AlignMent align, const char *text, uint8_t
// // font,
// //                             Colors txt, Colors bg) {
// //   if (!text) {
// //     ESP_LOGE("GUI", "Null pointer passed!");
// //     return;
// //   }

// //   int16_t textWidth = canvas_->textWidth(text, font);
// //   int16_t textHeight = canvas_->fontHeight(font);

// //   auto [x, y] = calculateAlignment(align, textWidth, textHeight);
// //   canvas_->setTextColor(static_cast<uint16_t>(txt),
// //   static_cast<uint16_t>(bg)); canvas_->drawString(text, x, y, font);
// // }
