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
      layer_1(std::make_unique<TFT_eSprite>(canvas_.get())),
      layer_2(std::make_unique<TFT_eSprite>(canvas_.get())),
      current_process_(GPSU::ProcessType::ANY) {}

//------------------------------------------------------------------------------
// Initialisation function.
//------------------------------------------------------------------------------

void Display::init() {
  if (!initialised) {

    canvas_->init();
    int16_t width = canvas_->width();
    int16_t height = canvas_->height();

    bg_->createSprite(width, height);
    label_->createSprite(width - TOP_MARGIN_PX, height - BOTTOM_MARGIN_PX);

    displayQueue = xQueueCreate(10, sizeof(Command));
    xTaskCreate(&Display::display_loop, "display_loop", 4196, this, 1, NULL);
    initialised = true;
  }
}
void Display::setCursorIndex(size_t index) { cursorIndex_ = index; }
void Display::reset_rotation() {
  canvas_->setRotation(0);
  rotation_ = 0;
}
void Display::change_to_horizontal() {
  rotation_ = 1;
  canvas_->setRotation(1);
}

void Display::deinitProcessFlags() {

  setup_traffic = false;
  setup_waterlevel = false;
  setup_stepper = false;
  setup_counter = false;
}
void Display::deleteSprites() {

  if (bg_->created()) {
    bg_->deleteSprite();
  }
  if (label_->created()) {
    label_->deleteSprite();
  }
  if (layer_1->created()) {
    layer_1->deleteSprite();
  }
  if (layer_2->created()) {
    layer_2->deleteSprite();
  }
}
void Display::createSprites() {
  int16_t width = canvas_->width();
  int16_t height = canvas_->height();
  if (bg_ && bg_->created()) {
    bg_->deleteSprite();
  }
  if (layer_1 && layer_1->created()) {
    layer_1->deleteSprite();
  }
  if (layer_2 && layer_2->created()) {
    layer_2->deleteSprite();
  }
  if (setup_counter) {
    bg_->createSprite(240, 133);
  } else {
    bg_->createSprite(width, height);
  }

  label_->createSprite(width - TOP_MARGIN_PX, height - BOTTOM_MARGIN_PX);

  if (setup_traffic) {
    layer_1->createSprite(IMG_WIDTH, IMG_HEIGHT);
    layer_1->setSwapBytes(true);
    LOG::DEBUG("GUI", "Traffic Light layer_1: %dx%d\n", IMG_WIDTH, IMG_HEIGHT);
  }
  if (setup_waterlevel) {
    layer_2->createSprite(FRAME_WIDTH, FRAME_HEIGHT);
  }
  if (setup_stepper) {
    layer_1->createSprite(IMG2_WIDTH, IMG2_HEIGHT);
    layer_1->setSwapBytes(true);
    layer_2->createSprite(IMG2_WIDTH, IMG2_HEIGHT);
    layer_2->setSwapBytes(true);
  }
  if (setup_counter) {
    bg_->setSwapBytes(true);
    layer_1->createSprite(width, height - 50);
    layer_1->setSwapBytes(true);
    layer_2->createSprite(width, height - 50);
    layer_2->setSwapBytes(true);
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
      case CmdType::SHOW_MENU:
        self->current_process_ = GPSU::ProcessType::ANY;
        self->setCursorIndex(cmd.cursor.index);
        self->reset_rotation();
        self->deleteSprites();
        self->createSprites();
        self->deinitProcessFlags();
        self->showMenu();
        break;
      case CmdType::SHOW_PROCESS_SCREEN:

        self->current_process_ = cmd.process_type;
        self->showProcessScreen(cmd.process_type);
        break;
      case CmdType::UPDATE_TRAFFIC_LIGHT:
        if (self->current_process_ == GPSU::ProcessType::TRAFFIC_LIGHT) {
          self->updateTrafficLight(cmd);
        }
        break;
      case CmdType::UPDATE_WATER_LEVEL:
        if (self->current_process_ == GPSU::ProcessType::WATER_LEVEL) {
          // self->updateWaterLevelDisplay(cmd.water_level_state,
          // cmd.analog_ch0);
        }
        break;
      case CmdType::UPDATE_STEPPER:
        if (self->current_process_ == GPSU::ProcessType::STEPPER_MOTOR) {
          // self->updateStepperDisplay(cmd.analog_ch0, cmd.analog_ch1);
        }
        break;
      case CmdType::UPDATE_COUNTER:
        if (self->current_process_ == GPSU::ProcessType::OBJECT_COUNTER) {
          self->updateObjectCounter(cmd);
        }
        break;
      }
    }
  }
}
//------------------------------------------------------------------------------
// Runs a process based on the ProcessType.
//------------------------------------------------------------------------------
void Display::prepare_assets(GPSU::ProcessType type) {
  current_process_ = type;

  switch (type) {
  case GPSU::ProcessType::TRAFFIC_LIGHT:
    LOG::DEBUG("GUI", "Preparing traffic light assets");
    deinitProcessFlags();
    setup_traffic = true;
    deleteSprites();
    createSprites();

    break;
  case GPSU::ProcessType::WATER_LEVEL:
    LOG::DEBUG("GUI", "Preparing  water level assets");
    deinitProcessFlags();
    setup_waterlevel = true;
    deleteSprites();
    createSprites();

    break;
  case GPSU::ProcessType::STEPPER_MOTOR:
    LOG::DEBUG("GUI", "Preparing  stepper motor assets");
    deinitProcessFlags();
    setup_stepper = true;
    deleteSprites();
    createSprites();

    break;
  case GPSU::ProcessType::STATE_MACHINE:
    LOG::DEBUG("GUI", "Preparing  state machine assets");
    deinitProcessFlags();

    deleteSprites();
    createSprites();

    break;
  case GPSU::ProcessType::OBJECT_COUNTER:
    LOG::DEBUG("GUI", "Preparing  item counter assets");
    deinitProcessFlags();
    setup_counter = true;
    change_to_horizontal();
    deleteSprites();
    createSprites();

    break;
  case GPSU::ProcessType::MOTOR_CONTROL:
    LOG::DEBUG("GUI", "Preparing  motor control assets");
    deleteSprites();
    createSprites();

    break;
  default:
    LOG::DEBUG("GUI", "Unknown process");
    break;
  }
}
// StartUp menu of GUI
void Display::showMenu() {
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

void Display::showProcessScreen(GPSU::ProcessType type) {
  //
  if (type == GPSU::ProcessType::OBJECT_COUNTER) {
    change_to_horizontal();
  } else {
    reset_rotation();
  }

  prepare_assets(type);

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

  bg_->pushSprite(0, 0);
}

void Display::processScreenSetup() {

  bg_->fillSprite(TFT_BLACK); // reset
  if (setup_counter) {
    bg_->fillSprite(static_cast<uint16_t>(Colors::black));
    bg_->pushImage(0, 0, 240, 130, Asset::plant2);
  } else if (setup_traffic) {
    bg_->fillSprite(static_cast<uint16_t>(Colors::black));
    bg_->setSwapBytes(true);
    bg_->pushImage(0, 0, 130, 240, Asset::road);
  } else {
    bg_->fillSprite(static_cast<uint16_t>(Colors::logo));
  }

  label_->fillSprite(TFT_BLACK);
  label_->setTextColor(static_cast<uint16_t>(Colors::white),
                       static_cast<uint16_t>(Colors::black));
  // layer_1->fillSprite(TFT_BLACK);
  if (setup_counter) {
    layer_1->fillSprite(TFT_BLACK); // Black for counters
  } else {
    layer_1->fillSprite(static_cast<uint16_t>(Colors::logo));
  }

  if (setup_waterlevel || setup_stepper || setup_counter) {
    layer_2->fillSprite(TFT_BLACK);
  }
}
void Display::processScreenExecute(int angle) {
  if (setup_traffic) {
    layer_1->pushToSprite(bg_.get(), 0, IMAGE_TOP_PX,
                          static_cast<uint16_t>(Colors::black));
  } else {
    layer_1->pushToSprite(bg_.get(), 0, IMAGE_TOP_PX,
                          static_cast<uint16_t>(Colors::black));
  }

  if (setup_stepper) {
    canvas_->setPivot(IMG2_WIDTH / 2, IMG2_HEIGHT / 2);
    layer_2->pushRotated(bg_.get(), angle,
                         static_cast<uint16_t>(Colors::black));
  }
  if (setup_waterlevel) {
    layer_2->pushToSprite(bg_.get(), 5, FRAME_TOP_PX,
                          static_cast<uint16_t>(Colors::black));
  }
  label_->pushToSprite(bg_.get(), LABEL_LEFT_PX, LABEL_TOP_PX,
                       static_cast<uint16_t>(Colors::black));

  bg_->pushSprite(0, 0);
}

void Display::updateTrafficLight(Command cmd) {
  processScreenSetup();
  label_->drawString("TRAFFIC_LIGHT", 5, 5, MENU_FONT);
  TrafficLight::State state = cmd.contexts.tl_context.curr;
  switch (state) {
  case TrafficLight::State::INIT: // Init
    layer_1->pushImage(0, 0, IMG_WIDTH, IMG_HEIGHT, Asset::blank_traffic);
    break;
  case TrafficLight::State::RED: // Red
    layer_1->pushImage(0, 0, IMG_WIDTH, IMG_HEIGHT, Asset::traffic_red);
    break;
  case TrafficLight::State::YELLOW: // Yellow
    layer_1->pushImage(0, 0, IMG_WIDTH, IMG_HEIGHT, Asset::traffic_yellow);
    break;
  case TrafficLight::State::GREEN: // Green
    layer_1->pushImage(0, 0, IMG_WIDTH, IMG_HEIGHT, Asset::traffic_green);
    break;
  case TrafficLight::State::FAULT: // Green
    layer_1->pushImage(0, 0, IMG_WIDTH, IMG_HEIGHT, Asset::blank_traffic);
    break;
    // default:
    //   layer_1->pushImage(0, 0, IMG_WIDTH, IMG_HEIGHT, Asset::blank_traffic);
    //   break;
  }
  processScreenExecute();
}

void Display::updateObjectCounter(Command cmd) {
  processScreenSetup();
  label_->setTextColor(TFT_RED, TFT_BLACK);
  label_->drawString("OBJECT_COUNTER", 5, 5, MENU_FONT);
  ObjectCounter::State state = cmd.contexts.oc_context.curr;
  const char *state_string = GPSU::Util::ToString::OCState(state);
  label_->drawString(state_string, 5, 20, MENU_FONT);

  // Draw conveyor background
  layer_1->fillSmoothRoundRect(0, 55, 240, 30, 10, TFT_BLUE, TFT_BLACK);
  layer_1->fillSmoothRoundRect(0, 60, 230, 20, 10, TFT_DARKGREY, TFT_BLACK);

  // Gradient::drawCustomGradient<50>(layer_1.get(), 0, 60, 230, 20,
  //                                  Gradient::lines);

  // Clear previous items from layer_2
  // layer_2->fillSprite(TFT_BLACK);

  // Conveyor dimensions
  const uint16_t conveyor_pixel_width = 230; // Matches gradient width
  const uint16_t conveyor_length_mm =
      cmd.contexts.oc_context.config.conv_length;
  Serial.printf("item count =%d\n", cmd.contexts.oc_context.obj_cnt);
  // Draw each item
  for (uint8_t i = 0; i < cmd.contexts.oc_context.obj_cnt; ++i) {
    const auto &obj = cmd.contexts.oc_context.items[i];
    Items::State state = obj.state;
    // Scale position and length to pixels
    uint16_t scaled_x = (obj.x_pos / 4);
    uint16_t object_length_px = 12;
    int id = obj.id;
    Serial.printf("item id%d position = %d mm\n", id, scaled_x);

    // Position item on conveyor (adjust y as needed)
    uint16_t y_pos = 20; // Align with conveyor's y=60 in layer_1 (60 - 55 = 5)
    // layer_2->fillRect(scaled_x, y_pos, 30, 30, TFT_BLUE);

    drawData data = {scaled_x, y_pos, 30, 30, id};
    // label_->setTextColor(TFT_RED, TFT_BLACK);
    //  label_->drawString(String(data.id), data.x + 2, 70, 2);
    drawBox(layer_2.get(), data, state);
  }

  // Draw layers to main display
  layer_2->pushToSprite(bg_.get(), 0, IMAGE_TOP_PX,
                        static_cast<uint16_t>(Colors::black));
  processScreenExecute();
}

void Display::drawBox(TFT_eSprite *sprite, drawData &data, Items::State state) {
  using State = Items::State;
  bool draw_object = false;
  uint32_t color = 0;
  const uint16_t *imgdata = nullptr;

  switch (state) {
  case State::INIT:
    break;
  case State::PLACED:
    color = TFT_BLUE;
    draw_object = true;

    break;
  case State::SENSED:
    color = TFT_YELLOW;
    draw_object = true;

    break;
  case State::ARRIVAL:
    color = TFT_DARKGREEN;
    draw_object = true;

    break;
  case State::PICKED:
    color = TFT_DARKGREY;
    draw_object = true;

    break;
  case State::FAILED:
    color = TFT_RED;
    draw_object = true;

    break;
  default:
    break;
  }
  if (draw_object) {
    // sprite->fillRect(data.x, data.y, data.w, data.h, color);
    sprite->pushImage(data.x, data.y, 40, 40, Asset::box);
  }
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
//   layer_2->fillRoundRect(tankX, tankY, FRMAE_WIDTH, FRAME_HEIGHT,
//   TANK_RADIUS,
//                         TFT_WHITE);
//   layer_2->fillRoundRect(tankX + TANK_BORDER_THK, tankY + TANK_BORDER_THK,
//                         TANK_WIDTH, TANK_HEIGHT, TANK_RADIUS, TFT_BLACK);
//   // Draw water
//   layer_2->fillRect(tankX + TANK_BORDER_THK, waterY, TANK_WIDTH,
//                    waterPixelHeight, waterColor);

//   processScreenExecute();
// }
void Display::updateStepperDisplay(const int dir, int step) {
  processScreenSetup();

  label_->drawString("Stepper-Motor", 0, 0, MENU_FONT);
  layer_1->pushImage(0, 0, IMG2_WIDTH, IMG2_HEIGHT,
                     Asset::stepper); // 130X130

  int stator_length = 80;
  int stator_width = 25;
  int stator_x = (IMG2_WIDTH - stator_length) / 2;
  int stator_y = (IMG2_HEIGHT - stator_width) / 2;

  layer_2->fillRoundRect(stator_x, stator_y, stator_length, stator_width, 20,
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

TFT_eSprite &Display::Sprite() { return *layer_1; }
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
