#include "IO_Controller.hpp"
#include "MCPDevice.hpp"
#include "PulseCounter.hpp"
#include "RotaryEncoder.hpp"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "oil.h"
#include <Arduino.h>
#include <ModbusClientTCP.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <WifiClient.h>

TaskHandle_t runTaskhandle = nullptr;

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite img = TFT_eSprite(&tft);

const int pwmFreq = 5000;
const int pwmResolution = 8;
const int pwmLedChannelTFT = 0;

#define green 0x33F0
#define gray 0xBDD7
double rad = 0.01745;

int sx = 67;
int sy = 76;
int angle = 0;
float x[360];
float y[360];
int r = 38;

int sx2 = 34;
int sy2 = 170;
int angle2 = 150;
float x2[360];
float y2[360];
int r2 = 22;

int sx3 = 106;
int sy3 = 204;
int angle3 = 100;
float x3[360];
float y3[360];
int r3 = 20;

int n = 12;
int fuelA = 0;
int tempA = 0;
int rpmA = 0;
int percent = 0;

static int send_req = 0;

gpio_num_t pinA = GPIO_NUM_37; // GPIO pin for Channel A
gpio_num_t pinB = GPIO_NUM_38;

// COMPONENT::Encoder encoder = COMPONENT::Encoder(4, pinA, pinB);
COMPONENT::pcnt_range_t ranges = COMPONENT::pcnt_range_t{-100, 100};
COMPONENT::PulseCounter encoder = COMPONENT::PulseCounter();

// MCP::MCPDevice<MCP::MCP_23X17::REG, MCP::MCP_MODEL::MCP23017> device;

COMPONENT::MCPDevice expander(MCP::MCP_MODEL::MCP23017);
void RunTask(void *param);
void setup() {

  Serial.begin(115200);
  tft.init();
  delay(1000);
  // auto &controller = GPSU_CORE::IO_Controller::getInstance(
  //     GPSU_CORE::Process::OBJECT_COUNTER, 500, 0x48);

  // encoder.begin();
  //  encoder.setup(readEncoderISR);
  bool circleValues = true;
  // encoder.setBoundaries(0, 10, circleValues);
  encoder.attach(pinA, pinB, ranges, GPSU_CORE::EncoderType::FULL);
  expander.dumpRegisters();
  delay(1000);
  // expander.cntrlRegA->separateBanks<MCP::REG::IOCON>();
  delay(2000);
  expander.pinMode(MCP::PORT::GPIOA, OUTPUT_OPEN_DRAIN);

  delay(1000);
  xTaskCreatePinnedToCore(RunTask, "RunTask", 4196, &expander, 2,
                          &runTaskhandle, 0);

  // pinMode(12,OUTPUT);
  // digitalWrite(12,1);

  // ledcSetup(pwmLedChannelTFT, pwmFreq, pwmResolution);
  // ledcAttachPin(5, pwmLedChannelTFT);
  // ledcWrite(pwmLedChannelTFT, 100);

  tft.fillScreen(TFT_WHITE);
  img.createSprite(135, 240);
  img.setTextDatum(4);
  img.setSwapBytes(true);

  img.setFreeFont(&FreeSansBold9pt7b);

  int i = 0;
  int a = 136;

  while (a != 44) {
    x[i] = r * cos(rad * a) + sx;
    y[i] = r * sin(rad * a) + sy;
    x2[i] = r2 * cos(rad * a) + sx2;
    y2[i] = r2 * sin(rad * a) + sy2;
    x3[i] = r3 * cos(rad * a) + sx3;
    y3[i] = r3 * sin(rad * a) + sy3;

    i++;
    a++;
    if (a == 360)
      a = 0;
  }

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
void loop() {

  Serial.println("....");
  delay(2000);
}
// void loop() {
//   // rpmA = map(analogRead(rpm_pin), 0, 4002, 0, 150);
//   // fuelA = map(analogRead(fuelgauge_pin), 0, 3890, 0, 100);
//   // tempA = map(analogRead(temperature_pin), 0, 3867, 0, 80);
//   // angle++;
//   // if(angle==266)

//   // angle2=angle2+2;;
//   // if(angle2==266)

//   // angle3++;
//   // if(angle3==266)

//   angle = 0;
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
//   Serial.println("....");
//   Serial.printf("count is %llu \n", encoder.getCount());
//   angle = encoder.getCount();
//   expander.gpioBankA->getPinStates();
//   expander.gpioBankB->setPinState(MCP::PIN::PIN11, true);
//   delay(100);
//   expander.gpioBankB->setPinState(MCP::PIN::PIN11, false);
//   expander.cntrlRegA->separateBanks<MCP::REG::IOCON>();
// }

void RunTask(void *param) {
  COMPONENT::MCPDevice *expander = static_cast<COMPONENT::MCPDevice *>(param);
  while (true) {
    Serial.println("....MAIN TASK......");
    uint8_t mask = 0b00011110;
    expander->gpioBankA->setPinState(mask, true);
    // send_req += 1;

    // expander->gpioBankA->setPinState(mask, false);
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t value = expander->gpioBankA->getPinState();
    Serial.printf("pins value is %d", value);
    vTaskDelay(pdMS_TO_TICKS(10));
    send_req += 1;
    if (send_req == 2) {
      send_req = 0;
      expander->gpioBankA->setPinState(mask, false);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    value = expander->gpioBankA->getPinState();
    Serial.printf("pins value is %d", value);
    vTaskDelay(pdMS_TO_TICKS(10));

    Serial.printf("Total send request=%d\n", send_req);
    Serial.println("..........");
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  vTaskDelete(NULL);
}
