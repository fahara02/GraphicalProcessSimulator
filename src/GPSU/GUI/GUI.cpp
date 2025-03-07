#include "GUI/GUI.hpp"

#include "Utility/gpsuUtility.hpp"
#include "esp_log.h"
namespace GUI {
StackType_t Display::sharedStack[2048]; // Define the stack array
StaticTask_t Display::sharedTCB;        // Define the TCB
} // namespace GUI
namespace GUI {
gpio_num_t pinA = GPIO_NUM_37;
gpio_num_t pinB = GPIO_NUM_38;
gpio_num_t btn = GPIO_NUM_32;

const MenuItem Display::menuItems[] = {
    {"TRAFFIC_LIGHT", Display::processTrafficLight},
    {"WATER_LEVEL", Display::processWaterLevel},
    {"STEPPER_MOTOR", Display::processStepperMotor},
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
      label_(std::make_unique<TFT_eSprite>(canvas_.get())),
      analog_(std::make_unique<TFT_eSprite>(canvas_.get())),
      img_(std::make_unique<TFT_eSprite>(canvas_.get())),
      imgRotor_(std::make_unique<TFT_eSprite>(canvas_.get())),

      frame_(std::make_unique<TFT_eSprite>(canvas_.get())),
      menu_(std::make_unique<MenuSelector>(menuItems, MENU_ITEM_COUNT, 4, pinA,
                                           pinB, btn)),
      current_process_(GPSU::ProcessType::ANY) {}

//------------------------------------------------------------------------------
// Initialisation function.
//------------------------------------------------------------------------------

void Display::init() {
  if (!initialised) {
    menu_->set_selection_changed_Cb([](size_t index) {
      DisplayCommand cmd;
      cmd.type = DisplayCommandType::SHOW_MENU;
      getInstance().sendDisplayCommand(cmd);
      if (getInstance().current_process_ == GPSU::ProcessType::ANY) {
        Display &display = getInstance();
        if (display.processTaskHandle != NULL) {
          vTaskDelete(display.processTaskHandle);
          display.processTaskHandle = NULL;
        }
      }
    });
    menu_->set_item_selected_cb([](size_t index) {
      menuItems[index].action(); // Sets current_process_
      DisplayCommand cmd;
      cmd.type = DisplayCommandType::SHOW_PROCESS_SCREEN;
      cmd.process_type = getInstance().current_process_;
      getInstance().sendDisplayCommand(cmd);
    });
    menu_->init();
    canvas_->init();
    bg_->createSprite(MAX_WIDTH, MAX_HEIGHT);
    label_->createSprite(MAX_WIDTH - TOP_MARGIN_PX,
                         MAX_HEIGHT - BOTTOM_MARGIN_PX);
    // img_->createSprite(IMG_WIDTH, IMG_HEIGHT);
    // imgRotor_->createSprite(IMG2_WIDTH, IMG2_HEIGHT);

    // img_->setSwapBytes(true);
    // imgRotor_->setSwapBytes(true);

    // frame_->createSprite(FRMAE_WIDTH, FRAME_HEIGHT);

    // Initialize display queue and task
    displayQueue = xQueueCreate(10, sizeof(DisplayCommand));
    xTaskCreate(&Display::display_loop, "display_loop", 2048, this, 1, NULL);

    // Show initial menu
    DisplayCommand cmd;
    cmd.type = DisplayCommandType::SHOW_MENU;
    sendDisplayCommand(cmd);

    initialised = true;
  }
}
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
void Display::sendDisplayCommand(const DisplayCommand &cmd) {
  xQueueSend(displayQueue, &cmd, portMAX_DELAY);
}
void Display::display_loop(void *param) {
  Display *self = static_cast<Display *>(param);
  DisplayCommand cmd;
  while (true) {
    if (xQueueReceive(self->displayQueue, &cmd, portMAX_DELAY) == pdPASS) {
      switch (cmd.type) {
      case DisplayCommandType::SHOW_MENU:
        self->showMenu();
        break;
      case DisplayCommandType::SHOW_PROCESS_SCREEN:
        self->current_process_ = cmd.process_type;

        self->showProcessScreen(cmd.process_type);
        break;
      case DisplayCommandType::UPDATE_TRAFFIC_LIGHT:
        if (self->current_process_ == GPSU::ProcessType::TRAFFIC_LIGHT) {

          self->updateTrafficLightDisplay(cmd.traffic_light_state);
        }
        break;
      case DisplayCommandType::UPDATE_WATER_LEVEL:
        if (self->current_process_ == GPSU::ProcessType::WATER_LEVEL) {

          self->updateWaterLevelDisplay(cmd.water_level_state, cmd.analog_ch0);
        }
        break;
      case DisplayCommandType::UPDATE_STEPPER:
        if (self->current_process_ ==
            GPSU::ProcessType::STEPPER_MOTOR_CONTROL) {

          self->updateStepperDisplay(cmd.analog_ch0, cmd.analog_ch1);
        }
        break;
      }
    }
  }
}
void Display::showProcessScreen(GPSU::ProcessType type) {
  bg_->fillSprite(static_cast<uint16_t>(Colors::logo));
  const int16_t fontSize = MENU_FONT;
  const char *label = "";
  label_->setTextColor(static_cast<uint16_t>(Colors::main),
                       static_cast<uint16_t>(Colors::black));
  // Determine the label and set up the label_ sprite
  for (size_t i = 0; i < MENU_ITEM_COUNT; ++i) {
    if (menuItems[i].action == processTrafficLight &&
        type == GPSU::ProcessType::TRAFFIC_LIGHT) {
      label = menuItems[i].label;
      // Set text color for TRAFFIC_LIGHT

    } else if (menuItems[i].action == processWaterLevel &&
               type == GPSU::ProcessType::WATER_LEVEL) {
      label = menuItems[i].label;

    } else if (menuItems[i].action == processStepperMotor &&
               type == GPSU::ProcessType::STEPPER_MOTOR_CONTROL) {
      label = menuItems[i].label;

    } else if (menuItems[i].action == processStateMachine &&
               type == GPSU::ProcessType::STATE_MACHINE) {
      label = menuItems[i].label;

    } else if (menuItems[i].action == processObjectCounter &&
               type == GPSU::ProcessType::OBJECT_COUNTER) {
      label = menuItems[i].label;

    } else if (menuItems[i].action == processMotorControl &&
               type == GPSU::ProcessType::MOTOR_CONTROl) {
      label = menuItems[i].label;
    }
  }
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
void Display::startProcess(GPSU::ProcessType type) {
  // Delete the existing task, if any
  if (processTaskHandle != NULL) {
    vTaskDelete(processTaskHandle);
    processTaskHandle = NULL; // Clear the handle
  }

  // Create the new task using the shared static memory
  if (type == GPSU::ProcessType::TRAFFIC_LIGHT) {
    processTaskHandle = xTaskCreateStatic(traffic_light_task, // Task function
                                          "traffic_task",     // Task name
                                          2048,               // Stack depth
                                          NULL,               // Parameters
                                          1,                  // Priority
                                          sharedStack,        // Shared stack
                                          &sharedTCB          // Shared TCB
    );
  } else if (type == GPSU::ProcessType::WATER_LEVEL) {
    processTaskHandle = xTaskCreateStatic(water_level_task, // Task function
                                          "water_level",    // Task name
                                          2048,             // Stack depth
                                          NULL,             // Parameters
                                          1,                // Priority
                                          sharedStack,      // Shared stack
                                          &sharedTCB        // Shared TCB
    );
  } else if (type == GPSU::ProcessType::STEPPER_MOTOR_CONTROL) {
    processTaskHandle = xTaskCreateStatic(stepper_task,    // Task function
                                          "stepper_motor", // Task name
                                          2048,            // Stack depth
                                          NULL,            // Parameters
                                          1,               // Priority
                                          sharedStack,     // Shared stack
                                          &sharedTCB       // Shared TCB
    );
  }
}

void Display::traffic_light_task(void *param) {
  int state = 0;
  while (true) {
    DisplayCommand cmd;
    cmd.type = DisplayCommandType::UPDATE_TRAFFIC_LIGHT;
    cmd.traffic_light_state = state;
    Display::getInstance().sendDisplayCommand(cmd);
    state = (state + 1) % 3;         // Cycle through 0, 1, 2
    vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
  }
}

void Display::water_level_task(void *param) {
  int level = 0;
  while (true) {
    int state = 0;
    state =
        Display::getInstance().tank_state.load(); // Update state each iteration

    if (state == 1) { // Filling

      level = (level >= 100) ? 100 : level + 1;
      if (level == 100)
        state = 2;
      vTaskDelay(pdMS_TO_TICKS(100));
    } else if (state == 2) { // Full

      vTaskDelay(pdMS_TO_TICKS(1000));
      state = 0;             // Transition to drain
    } else if (state == 0) { // Draining

      level = (level <= 0) ? 0 : level - 1;
      if (level == 0) {
        state = 1;
      }

      vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (state >= 0 || state <= 3) {

      Display::getInstance().tank_state.store(state);
      DisplayCommand cmd;
      cmd.type = DisplayCommandType::UPDATE_WATER_LEVEL;
      cmd.water_level_state = state;
      cmd.analog_ch0 = level;
      Display::getInstance().sendDisplayCommand(cmd);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}
void Display::stepper_task(void *param) {
  while (true) {
    int dir = 1;
    int step = 0;
    step = Display::getInstance().stepper_step;
    step += 1;
    if (step > 360) {
      step = 1;
      dir = dir * (-1);
    }

    DisplayCommand cmd;
    cmd.type = DisplayCommandType::UPDATE_STEPPER;
    Display::getInstance().stepper_step = step;
    cmd.stteper_motor_state = 1; // running
    cmd.analog_ch0 = dir;
    cmd.analog_ch1 = step;
    Display::getInstance().sendDisplayCommand(cmd);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void Display::processScreenSetup() {
  bg_->fillSprite(TFT_BLACK);
  bg_->fillSprite(static_cast<uint16_t>(Colors::logo));
  img_->fillSprite(TFT_BLACK);
  imgRotor_->fillSprite(TFT_BLACK);

  frame_->fillSprite(TFT_BLACK);
  label_->fillSprite(TFT_BLACK);
  label_->setTextColor(static_cast<uint16_t>(Colors::white),
                       static_cast<uint16_t>(Colors::black));
}
void Display::processScreenExecute(int angle) {

  img_->pushToSprite(bg_.get(), 0, IMAGE_TOP_PX,
                     static_cast<uint16_t>(Colors::black));
  canvas_->setPivot(IMG2_WIDTH / 2, IMG2_HEIGHT / 2);
  // imgStator_->pushRotated(bg_.get(), angle,
  //                         static_cast<uint16_t>(Colors::black));

  imgRotor_->pushRotated(bg_.get(), angle,
                         static_cast<uint16_t>(Colors::black));

  frame_->pushToSprite(bg_.get(), 5, FRAME_TOP_PX,
                       static_cast<uint16_t>(Colors::black));
  label_->pushToSprite(bg_.get(), LABEL_LEFT_PX, LABEL_TOP_PX,
                       static_cast<uint16_t>(Colors::black));
  bg_->pushSprite(0, 0);
}

void Display::updateTrafficLightDisplay(int state) {
  processScreenSetup();
  label_->drawString("TRAFFIC_LIGHT", 5, 5, MENU_FONT);
  switch (state) {
  case 0: // Red
    img_->pushImage(0, 0, IMG_WIDTH, IMG_HEIGHT, Asset::traffic_red);
    break;
  case 1: // Yellow
    img_->pushImage(0, 0, IMG_WIDTH, IMG_HEIGHT, Asset::traffic_yellow);
    break;
  case 2: // Green
    img_->pushImage(0, 0, IMG_WIDTH, IMG_HEIGHT, Asset::traffic_green);
    break;
  }
  processScreenExecute();
}
void Display::updateWaterLevelDisplay(const int state, int level) {
  processScreenSetup();
  // Draw labels
  label_->drawString("Water-Level", 0, 0, MENU_FONT);

  uint16_t waterColor = TFT_BLUE;
  const int tankX = 0; // Centered within FRAME_WIDTH
  const int tankY = 0;
  // Calculate water height
  int waterPixelHeight = (level * TANK_HEIGHT) / 100;
  int waterY = tankY + TANK_BORDER_THK + (TANK_HEIGHT - waterPixelHeight);

  switch (state) {
  case 0:
    waterColor = TFT_BLUE;
    break;
  case 1:
    waterColor = TFT_CYAN;
    break;
  case 2:
    waterColor = TFT_GREEN;
    break;
  case 3:
    waterColor = TFT_RED;
    break;
  default:
    waterColor = TFT_YELLOW;
    break;
  }

  label_->drawString(GPSU::Util::ToString::TankState(state), 0, 20, MENU_FONT);
  uint16_t litre = (tank_capacity_litre * level) / 100;
  label_->setTextColor(TFT_WHITE);
  label_->drawString(String(litre), 0, 120, 7);

  // Draw thick tank border
  frame_->fillRoundRect(tankX, tankY, FRMAE_WIDTH, FRAME_HEIGHT, TANK_RADIUS,
                        TFT_WHITE);
  frame_->fillRoundRect(tankX + TANK_BORDER_THK, tankY + TANK_BORDER_THK,
                        TANK_WIDTH, TANK_HEIGHT, TANK_RADIUS, TFT_BLACK);
  // Draw water
  frame_->fillRect(tankX + TANK_BORDER_THK, waterY, TANK_WIDTH,
                   waterPixelHeight, waterColor);

  processScreenExecute();
}
void Display::updateStepperDisplay(const int dir, int step) {
  processScreenSetup();

  label_->drawString("Stepper-Motor", 0, 0, MENU_FONT);
  img_->pushImage(0, 0, IMG2_WIDTH, IMG2_HEIGHT,
                  Asset::stepper); // 130X130
  int pivot_x = IMG2_WIDTH / 2;
  int pivot_y = IMG2_HEIGHT / 2;
  int stator_length = 100;
  int stator_width = 25;
  int stator_x = (IMG2_WIDTH - stator_length) / 2;
  int stator_y = IMG2_HEIGHT / 2;

  imgRotor_->fillRect(stator_x, IMG2_HEIGHT / 2, stator_length, stator_width,
                      TFT_BLUE);

  // label_->drawString(String(step), 0, 20, 7);
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
// show_menu (//https://barth-dev.de/online/rgb565-color-picker/)
// https://www.iconarchive.com/show/farm-fresh-icons-by-fatcow/traffic-lights-green-icon.html
// https://www.cleanpng.com/png-traffic-light-indicating-different-types-of-traffi-7915271/download-png.html
// https://www.pngwing.com/en/free-png-yjwqm/download
// http: // www.rinkydinkelectronics.com/
// https://oleddisplay.squix.ch/#/home
//------------------------------------------------------------------------------
void Display::showMenu() {
  // Clear the screen with a black background
  current_process_ = GPSU::ProcessType::ANY;
  bg_->fillScreen(TFT_BLACK);

  // Get the index of the selected menu item
  size_t selected_index = menu_->get_selected_index();

  // Define constants for layout
  const int16_t padding = PADDING_PX;
  const int16_t fontSize = MENU_FONT;
  const int16_t textHeight = bg_->fontHeight(fontSize);
  const int16_t itemHeight = textHeight + MENU_VERTICAL_PADDING;

  // Calculate number of menu items
  size_t num_items = sizeof(menuItems) / sizeof(menuItems[0]);
  // Define frame padding
  const int16_t frame_padding = 10; // Adjustable padding around the frame
  // Calculate frame top and bottom positions
  int16_t y_frame_top = padding - frame_padding;
  if (y_frame_top < 0)
    y_frame_top = 0; // Ensure it doesn’t go off-screen
  int16_t y_frame_bottom =
      padding + (num_items + 1) * itemHeight + frame_padding;
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
  for (size_t i = 0; i < num_items; i++) {
    int16_t yPos = padding + (i + 1) * itemHeight; // Shifted position

    // Draw a background rectangle for the selected item
    if (i == selected_index) {
      bg_->fillRect(0, yPos, bg_->width(), itemHeight,
                    static_cast<uint16_t>(Colors::logo));
    }

    // Calculate text dimensions
    int16_t textWidth = bg_->textWidth(menuItems[i].label, fontSize);
    // Calculate x-position to center the text horizontally
    int16_t xPos = (bg_->width() - textWidth) / 2;
    // Adjust y-position to center the text vertically within the item height
    int16_t adjustedYPos = yPos + (itemHeight - textHeight) / 2;
    // Set text and background colors
    Colors textColor = Colors::main;
    Colors bgColor = (i == selected_index) ? Colors::logo : Colors::black;
    bg_->setTextColor(static_cast<uint16_t>(textColor),
                      static_cast<uint16_t>(bgColor));

    // Draw the menu item's text
    bg_->drawString(menuItems[i].label, xPos, adjustedYPos, fontSize);
  }

  // Push the sprite to the display
  bg_->pushSprite(0, 0);
}

void Display::showSubMenu() { Serial.println("not implemented yet"); }

//
//------------------------------------------------------------------------------
// Static callback functions for each menu item.
//------------------------------------------------------------------------------
void Display::processTrafficLight() {
  getInstance().run_process(GPSU::ProcessType::TRAFFIC_LIGHT);
}

void Display::processWaterLevel() {
  getInstance().run_process(GPSU::ProcessType::WATER_LEVEL);
}

void Display::processStepperMotor() {
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
    startProcess(type);
    Serial.println("Running traffic light process");
    break;
  case GPSU::ProcessType::WATER_LEVEL:
    deinitProcessFlags();
    setup_waterlevel = true;
    deleteSprites();
    createSprites();
    startProcess(type);
    Serial.println("Running water level process");
    break;
  case GPSU::ProcessType::STEPPER_MOTOR_CONTROL:
    deinitProcessFlags();
    setup_stepper = true;
    deleteSprites();
    createSprites();
    startProcess(type);
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

} // namespace GUI

// std::tuple<int16_t, int16_t>
//  Display::calculateAlignment(AlignMent align, int16_t Width, int16_t
//  Height) {
//    int16_t x = 0;
//    int16_t y = 0;

//   switch (align) {
//   case AlignMent::TOP_MIDDLE:
//     x = (MAX_WIDTH - Width) / 2;
//     y = TOP_MARGIN_PX;
//     break;

//   case AlignMent::CENTER_MIDDLE:
//     x = (MAX_WIDTH - Width) / 2;
//     y = (MAX_HEIGHT - Height) / 2;
//     break;

//   case AlignMent::LEFT_X:
//     x = LEFT_MARGIN_PX;
//     y = (MAX_HEIGHT - Height) / 2;
//     break;

//   case AlignMent::RIGHT_X:
//     x = MAX_WIDTH - Width - RIGHT_MARGIN_PX;
//     y = (MAX_HEIGHT - Height) / 2;
//     break;

//   case AlignMent::BOTTOM_MIDDLE:
//     x = (MAX_WIDTH - Width) / 2;
//     y = MAX_HEIGHT - Height - BOTTOM_MARGIN_PX;
//     break;

//   default:
//     ESP_LOGW("GUI", "Unknown alignment, defaulting to TOP_MIDDLE");
//     x = (MAX_WIDTH - Width) / 2;
//     y = TOP_MARGIN_PX;
//     break;
//   }

//   return std::make_tuple(x, y);
// }
// void Display::drawAlignText(AlignMent align, const char *text, uint8_t
// font,
//                             Colors txt, Colors bg) {
//   if (!text) {
//     ESP_LOGE("GUI", "Null pointer passed!");
//     return;
//   }

//   int16_t textWidth = canvas_->textWidth(text, font);
//   int16_t textHeight = canvas_->fontHeight(font);

//   auto [x, y] = calculateAlignment(align, textWidth, textHeight);
//   canvas_->setTextColor(static_cast<uint16_t>(txt),
//   static_cast<uint16_t>(bg)); canvas_->drawString(text, x, y, font);
// }
