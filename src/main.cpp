#include "FS.h"
#include "GUI/GUI.hpp"
#include "IO_Controller.hpp"
#include "Logger.hpp"
#include "MCP23017.hpp"
#include "MenuSelector.hpp"
#include "Process/Process.hpp"
#include "PulseCounter.hpp"
#include "RotaryEncoder.hpp"
#include "StateMachines/ObjectCounterSM.hpp"
#include "StateMachines/StepperMotorSM.hpp"
#include "StateMachines/TrafficLightSM.hpp"
#include "StateMachines/WaterLevelSM.hpp"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include <Arduino.h>
#include <ModbusClientTCP.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <WifiClient.h>
#include <eFSM.hpp>

gpio_num_t sda = GPIO_NUM_25;
gpio_num_t scl = GPIO_NUM_33;
gpio_num_t reset = GPIO_NUM_13;
TaskHandle_t runTaskhandle = nullptr;

void RunTask(void* param);
void TestTask(void* param);

// TrafficLight::Context context{
//     TrafficLight::State::INIT, // previous_state
//     {5000, 3000, 2000, false}, // config
//     {0},                       // data
//     {{0}, {false}},            // inputs
//     TrafficLight::Event::OK,   // Event
//     TrafficLight::Mode::AUTO   // Mode
// };

// SM::TrafficLightSM trafficLight(context, TrafficLight::State::INIT,
//                                 true); // Global item
// unsigned long lastMillis;      // Global variable to track last update time
ObjectCounter::State prevState; // Global variable to track previous state
								// SM::WaterLevelSM waterLevel;
GPSU::Process process;
// SM::ObjectCounterSM oc;
void setup()
{

	Serial.begin(115200);
	LOG::ENABLE_TIMESTAMPS();

	process.init();
	// oc.update();

	// Initialize traffic light with timeouts: 5s red, 3s green, 2s yellow

	// Perform initial update to transition from INIT to RED_STATE
	// delay(2000);
	// trafficLight.init();
	// delay(2000);
	// trafficLight.update();
	// waterLevel.update();
	//   Print initial state

	// Record initial time and state
	// lastMillis = millis();
	// prevState = trafficLight.current();
	// tft.init();
	// display.init();
	// display.showMenu();
	// delay(1000);
	// auto &controller = GPSU_CORE::IO_Controller::getInstance(
	//     GPSU_CORE::Process::OBJECT_COUNTER, 500, 0x48);

	// encoder.init();
	// encoder.set_callbacks(readEncoderISRcb);
	// bool circleValues = true;
	// encoder.set_range(0, 10, circleValues);

	// encoder.attach(pinA, pinB, ranges, GPSU_CORE::EncoderType::FULL);

	// expander.init();
	// expander.dumpRegisters();
	// delay(1000);
	// MCP::Settings setting;
	// setting.opMode = MCP::OperationMode::SequentialMode16;
	// expander.configure(setting);
	// delay(1000);

	// // expander.enableInterrupt();

	// expander.pinMode(OUTPUT_OPEN_DRAIN, GPA1, GPA2, GPA3, GPA4);
	// delay(1000);
	// expander.pinMode(MCP::PORT::GPIOB, INPUT);
	// delay(1000);
	// expander.invertInput(true, GPB1, GPB2, GPB3, GPB4);
	// delay(1000);
	// int sensorThershold = 12;
	// expander.setInterrupts(GPB1, GPB2, RISING);
	// expander.setInterrupts(GPB3, GPB4, RISING,
	//                        MCP::INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH);
	// expander.setInterrupts(GPB5, cb1, GPB6, cb2, RISING,
	//                        MCP::INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH);
	// expander.setInterrupts(GPB7, cb3, &sensorThershold, GPB0, cb4,
	//                        &sensorThershold, RISING,
	//                        MCP::INTR_OUTPUT_TYPE::INTR_ACTIVE_HIGH);
	// expander.dumpRegisters();

	// xTaskCreatePinnedToCore(TestTask, "TestTask", 4196, NULL, 2,
	// &runTaskhandle,
	//                         0);

	// pinMode(12,OUTPUT);
	// digitalWrite(12,1);

	// ledcSetup(pwmLedChannelTFT, pwmFreq, pwmResolution);
	// ledcAttachPin(5, pwmLedChannelTFT);
	// ledcWrite(pwmLedChannelTFT, 100);

	// menu.set_selection_changed_Cb(onSelectionChanged);
	// menu.set_item_selected_cb(onItemSelected);
	// menu.init();

	// tft.fillScreen(TFT_WHITE);
	// img.createSprite(135, 240);
	// img.setTextDatum(4);
	// img.setSwapBytes(true);

	// img.setFreeFont(&FreeSansBold9pt7b);

	// device.printRegisters();
	//  for (uint8_t i = static_cast<uint8_t>(MCP::MCP_23X17::REG::IODIR);
	//       i <= static_cast<uint8_t>(MCP::MCP_23X17::REG::OLAT); ++i) {
	//    MCP::MCP_23X17::REG reg = static_cast<MCP::MCP_23X17::REG>(i);

	//   Serial.printf("A PORT 0x%02X\n",
	//                 MCP::MCP_23X17::getRegisterAddress(reg, true, false));
	//   Serial.printf("B PORT 0x%02X\n",
	//                 MCP::MCP_23X17::getRegisterAddress(reg, true, true));
	// }
	// ledcSetup(pwmLedChannelTFT, pwmFreq, pwmResolution);
	// ledcAttachPin(TFT_BL, pwmLedChannelTFT);
	// ledcWrite(pwmLedChannelTFT, 100);
}
void loop()
{
	Serial.println("......");

	vTaskDelay(500);
}

// void TestTask(void *param) {
//   while (true) {
//     vTaskDelay(50);
//     // Update the state machine to check transitions
//     // oc.setAutoUpdate();
//     ObjectCounter::Command cmd = oc.updateData();
//     const char *exitCommand =
//         GPSU::Util::ToString::OCCommands(cmd.exit_command);
//     const char *entryCommand =
//         GPSU::Util::ToString::OCCommands(cmd.entry_command);
//     const char *State = GPSU::Util::ToString::OCState(oc.current());

//     if (cmd.check_exit) {
//       Serial.printf("exit command is %s for State %s \n", exitCommand,
//       State);
//     }
//     if (cmd.check_entry) {
//       Serial.printf("entry command is %s for State %s \n", entryCommand,
//       State);
//     }

//     // Check if the state has changed
//     ObjectCounter::State currentState = oc.current();
//     if (currentState != prevState) {
//       const char *current = GPSU::Util::ToString::OCState(oc.current());
//       const char *previous = GPSU::Util::ToString::OCState(oc.previous());

//       Serial.printf("State changed to: %s from state %s  int time %lu\n",
//                     current, previous, millis());

//       prevState = currentState;
//     }
//   }
//   vTaskDelete(NULL);
// }
// void TestTask(void *param) {
//   while (true) {
//     vTaskDelay(50);
//     unsigned long currentMillis = millis();
//     int delta = currentMillis - lastMillis;
//     TrafficLight::Inputs input;
//     // input.external_timer.delta_time_ms = delta;
//     // trafficLight.updateData(input);
//     lastMillis = currentMillis;

//     // Update the state machine to check transitions
//     TrafficLight::Command cmd = trafficLight.update();
//     const char *exitCommand =
//         GPSU::Util::ToString::TLCommands(cmd.exit_command);
//     const char *entryCommand =
//         GPSU::Util::ToString::TLCommands(cmd.entry_command);
//     const char *State =
//     GPSU::Util::ToString::TLState(trafficLight.current());

//     if (cmd.check_exit) {
//       Serial.printf("exit command is %s for State %s \n", exitCommand,
//       State);
//     }
//     if (cmd.check_entry) {
//       Serial.printf("entry command is %s for State %s \n", entryCommand,
//       State);
//     }

//     // Check if the state has changed
//     TrafficLight::State currentState = trafficLight.current();
//     if (currentState != prevState) {
//       const char *current =
//           GPSU::Util::ToString::TLState(trafficLight.current());
//       const char *previous =
//           GPSU::Util::ToString::TLState(trafficLight.previous());

//       Serial.printf("State changed to: %s from state %s in time %lu \n",
//                     current, previous, currentMillis);

//       prevState = currentState;
//     }
//   }
//   vTaskDelete(NULL);
// }
// void loop() {
//   int angle, angle2, angle3 = 0;

//   // rpmA = map(analogRead(rpm_pin), 0, 4002, 0, 150);
//   // fuelA = map(analogRead(fuelgauge_pin), 0, 3890, 0, 100);
//   // tempA = map(analogRead(temperature_pin), 0, 3867, 0, 80);
//   angle++;
//   if (angle == 266)

//     angle2 = angle2 + 2;

//   if (angle2 == 266)

//     angle3++;
//   if (angle3 == 266)

//     angle = 0;
//   angle2 = 0;
//   angle3 = 27;

//   img.fillSprite(TFT_WHITE);
//   img.fillCircle(sx + 52, sy - 32, 4, 0x9062);
//   img.fillCircle(sx + 52, sy - 18, 4, 0x9062);

//   img.drawCircle(sx, sy, r, gray);
//   img.drawCircle(sx, sy, r - 1, gray);
//   img.drawCircle(sx2, sy2, r2, gray);
//   img.fillRect(sx - r, sy + r - 12, 100, 20, TFT_WHITE);

//   img.fillRect(0, 0, 140, 22, green);
//   img.fillRect(0, 122, 68, 16, green);
//   img.fillRect(68, 160, 68, 16, green);
//   img.setTextColor(TFT_WHITE, green);
//   img.drawString("DASHBOARD", 40, 10, 2);
//   img.drawString("FUEL", 36, 130, 2);

//   img.drawString("TEMP", 36 + 68, 168, 2);

//   img.setTextColor(TFT_PURPLE, TFT_WHITE);
//   img.drawString("mph", sx, sy + 30, 2);
//   img.drawString("IDEA3D", 105, 136, 2);
//   img.drawString("bangladesh", 104, 150, 2);

//   img.drawString("check engine", 44, 230, 2);
//   img.setTextColor(TFT_BLACK, TFT_WHITE);

//   img.drawCircle(sx3, sy3, r3, gray);

//   for (int i = 0; i < angle; i++)
//     img.fillCircle(x[i], y[i], 3, green);

//   for (int i = 0; i < angle2; i++)
//     img.fillCircle(x2[i], y2[i], 2, green);

//   for (int i = 0; i < angle3; i++)
//     img.fillCircle(x3[i], y3[i], 2, green);

//   img.drawString(String(angle), sx, sy, 4);
//   img.setTextColor(TFT_PURPLE, TFT_WHITE);
//   img.drawString(String((int)angle2 / 10), sx2, sy2);
//   img.drawString(String((int)angle3 / 10), sx3, sy3);
//   img.pushImage(12, 198, 44, 17, oil);

//   img.pushSprite(0, 0);
// }

// void RunTask(void *param) {
//   MCP::MCPDevice *expander = static_cast<MCP::MCPDevice *>(param);
//   while (true) {
//     Serial.println("....MAIN TASK......");
//     // uint8_t readmask = 0;
//     // readmask = expander->digitalRead(GPB1, GPB2, GPB3, GPB4);

//     // vTaskDelay(pdMS_TO_TICKS(10));

//     // expander->digitalWrite(MCP::PORT::GPIOA, readmask, true);
//     // vTaskDelay(pdMS_TO_TICKS(10));

//     expander->digitalWrite(true, GPA1, GPA2, GPA3, GPA4);
//     send_req += 1;
//     // expander->gpioBankA->setPinState(mask, false);
//     vTaskDelay(pdMS_TO_TICKS(100));
//     uint8_t readmask = expander->digitalRead(GPA1, GPA2, GPA3, GPA4);
//     Serial.printf("pins value is %d", readmask);
//     vTaskDelay(pdMS_TO_TICKS(10));
//     send_req += 1;
//     if (send_req == 2) {
//       send_req = 0;
//       expander->digitalWrite(false, GPA1, GPA2, GPA3, GPA4);
//       vTaskDelay(pdMS_TO_TICKS(10));
//       expander->digitalRead(GPB1, GPB2, GPB3, GPB4);
//       Serial.printf("B ports value is %d", readmask);
//       vTaskDelay(pdMS_TO_TICKS(10));
//     }
//     readmask = expander->digitalRead(GPA1, GPA2, GPA3, GPA4);
//     vTaskDelay(pdMS_TO_TICKS(10));
//     Serial.printf("pins value is %d", readmask);

//     vTaskDelay(pdMS_TO_TICKS(10));

//     Serial.printf("Total send request=%d\n", send_req);
//     Serial.println("..........");
//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
//   vTaskDelete(NULL);
// }
