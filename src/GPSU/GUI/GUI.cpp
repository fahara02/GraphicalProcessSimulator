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
void Display::updateCursorIndex(size_t index) {
  cursorIndex_ = std::min(index, menuItemCount_ - 1);

  // Update scroll position
  if (cursorIndex_ < scrollOffset) {
    scrollOffset = cursorIndex_;
    needsRedraw = true;
  } else if (cursorIndex_ >= scrollOffset + visible_items) {
    scrollOffset = cursorIndex_ - visible_items + 1;
    needsRedraw = true;
  }
}
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

    layer_1->createSprite(width, height - 50);
    layer_1->setSwapBytes(false);
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
        self->updateCursorIndex(cmd.cursor.index);
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
  rotateMenu();
  if (needsRedraw) {
    calculateMenuMetrics();
    drawMenuFramework();
    needsRedraw = false;
  }
  drawMenuItems();
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

  const int16_t fontSize = MENU_FONT;
  const char *label = GPSU::Util::ToString::Process(type);
  label_->setTextColor(Colors::main, Colors::black);
  processScreenSetup();
  label_->fillSprite(Colors::black);
  label_->drawString(label, 0, 0, fontSize);

  label_->pushToSprite(bg_.get(), LABEL_LEFT_PX, LABEL_TOP_PX, Colors::black);

  bg_->pushSprite(0, 0);
}

void Display::processScreenSetup() {

  bg_->fillSprite(TFT_BLACK); // reset
  if (setup_counter) {
    bg_->unloadFont();

    bg_->fillScreen(Colors::black);
    bg_->fillSprite(Colors::white);
    bg_->setSwapBytes(true);
    bg_->pushImage(0, 80, 240, 69, Asset::conv_belt2);
    bg_->pushImage(55, 5, 40, 60, Asset::sensor);
  } else if (setup_traffic) {
    bg_->fillSprite(Colors::black);
    bg_->setSwapBytes(true);
  } else {
    bg_->fillSprite(Colors::logo);
  }

  label_->fillSprite(TFT_BLACK);
  label_->setTextColor(Colors::white, Colors::black);
  // layer_1->fillSprite(TFT_BLACK);
  if (setup_counter) {
    layer_1->fillSprite(TFT_BLACK); // Black for counters

  } else {
    layer_1->fillSprite(Colors::logo);
  }

  if (setup_waterlevel || setup_stepper || setup_counter) {
    layer_2->fillSprite(TFT_BLACK);
  }
}
void Display::processScreenExecute(int angle) {
  if (setup_traffic) {
    layer_1->pushToSprite(bg_.get(), 0, IMAGE_TOP_PX);
  } else if (setup_counter) {
    layer_1->pushToSprite(bg_.get(), 0, IMAGE_TOP_PX, Colors::black);
  } else {
    layer_1->pushToSprite(bg_.get(), 0, IMAGE_TOP_PX, Colors::black);
  }

  if (setup_stepper) {
    canvas_->setPivot(IMG2_WIDTH / 2, IMG2_HEIGHT / 2);
    layer_2->pushRotated(bg_.get(), angle, Colors::black);
  }
  if (setup_waterlevel) {
    layer_2->pushToSprite(bg_.get(), 5, FRAME_TOP_PX, Colors::black);
  }
  label_->pushToSprite(bg_.get(), LABEL_LEFT_PX, LABEL_TOP_PX, Colors::black);

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
  }
  processScreenExecute();
}

void Display::updateObjectCounter(Command cmd) {
  processScreenSetup();
  label_->unloadFont();
  label_->loadFont(NotoSansMonoSCB20);
  label_->setTextColor(swapBytes(Colors::logo), Colors::white);
  label_->drawString("OBJECT_COUNTER", 50, 5);
  ObjectCounter::State state = cmd.contexts.oc_context.curr;
  // const char *state_string = GPSU::Util::ToString::OCState(state);
  // label_->unloadFont();
  // label_->setFreeFont(&Orbitron_Medium_18);
  // label_->drawString(state_string, 5, 30);

  if (cmd.contexts.oc_context.obj_cnt > 0) {
    for (uint8_t i = 0; i < cmd.contexts.oc_context.obj_cnt; ++i) {
      const auto &obj = cmd.contexts.oc_context.items[i];
      Items::State state = obj.state;
      // Scale position and length to pixels
      uint16_t scaled_x = (obj.x_pos / 4);
      uint16_t scaled_y = (obj.y_pos);
      uint16_t box_size = 40;

      int id = obj.id;
      LOG::INFO("GUI", "item id %d in pos = %d mm\n", id, scaled_x);

      uint16_t y_pos = (10) - scaled_y;

      drawData data = {scaled_x, y_pos, box_size, box_size, id};
      drawBox(layer_1.get(), data, state);
    }
  }

  processScreenExecute();
}

void Display::drawBox(TFT_eSprite *sprite, drawData &data, Items::State state) {
  using State = Items::State;
  uint32_t fill_color = 0;
  uint32_t border_color = Colors::black;
  // sprite->setSwapBytes(true);

  switch (state) {
  case State::PLACED:
    fill_color = swapBytes(0x001F); // Dark blue
    break;
  case State::SENSED:
    fill_color = swapBytes(0xFFE0); // Yellow
    break;
  case State::ARRIVAL:
    fill_color = swapBytes(0x07E0); // Bright green
    break;
  case State::PICKED:
    fill_color = swapBytes(0x7BEF); // Mid-gray
    break;
  case State::FAILED:
    fill_color = swapBytes(0xF800); // Red
    break;
  default:
    return;
  }
  sprite->pushImage(data.x, data.y, data.w, data.h, Asset::box);
  // sprite->fillRoundRect(data.x, data.y, data.w, data.h, 3, fill_color);
  // sprite->drawRoundRect(data.x, data.y, data.w, data.h, 3, border_color);

  // sprite->drawFastHLine(data.x + 2, data.y + 2, data.w - 4,
  //                       lightenColor(fill_color));
  // sprite->drawFastVLine(data.x + 2, data.y + 2, data.h - 4,
  //                       lightenColor(fill_color));
  // sprite->drawFastHLine(data.x + 2, data.y + data.h - 3, data.w - 4,
  //                       darkenColor(fill_color));
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
