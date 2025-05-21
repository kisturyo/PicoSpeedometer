#include <Arduino.h>
#include <U8g2lib.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include "hardware/timer.h"
#include "main.hpp"

Adafruit_NeoPixel led = Adafruit_NeoPixel(WS2812_NUM, WS2812_PIN, NEO_GRB + NEO_KHZ800);

unsigned long saveCompleteTime = 0;           // 保存完成时间戳
const unsigned long saveBlinkDuration = 1000; // 保存后闪烁持续时间
bool isBlinking = false;                      // 是否处于保存后的闪烁状态

// EEPROM存储结构
struct SystemConfig {
  unsigned long totalDistance;                // 里程：m
  uint16_t wheelDiameter;                     // 车轮直径：mm
  float overspeedThreshold;                   // 超速阀值：km/h
  uint8_t magnetCount;                        // 磁铁数量（1-9）
  float maxSpeed;                             // 最大速度：km/h
  unsigned long totalTravelTime;              // 累计时间：s
};
SystemConfig config;                          // 初始化结构

// 全局变量
volatile unsigned long pulseCount = 0;        // 新增脉冲计数器
volatile uint32_t lastTriggerTime = 0;        // 上次触发中断时刻
volatile uint32_t pulseInterval = 0;          // 中断触发间隔
float currentSpeed = 0.0;                     // 当前速度
unsigned long lastUpdateTime = 0;             // 上次更新
bool needsSave = false;                       // 数据是否需要保存
uint32_t signleTravelTime = 0;                // 单次行驶时间：ms
uint32_t travelStartTime = 0;                 // 计时开始时间
uint32_t deltaTravelTime = 0;                 // 自行车启动到停止的间隔
bool isTraveling = false;                     // 是否正在计时
bool confirmReset = false;                    // 是否清零
bool isBuzzing = false;                       // 蜂鸣器状态标志
float totalDistanceFloat = 0.0;    	          // 高精度累计里程
float totalTravelTimeFloat = 0.0;  	          // 高精度累计时间
unsigned long dynamicDebounce = 100;          // 霍尔传感器动态消抖
bool isHallConnected = true;                  // 传感器连接状态
unsigned long hallCheckTime = 0;              // 连接状态变化时间戳
const unsigned long hallWaitTime = 2000;      // 霍尔传感器连接等待时长

// 平滑算法选择
enum FilterType { SLIDING_AVG, LIMITED_AVG, WEIGHTED_AVG, LOW_PASS, KALMAN, NONE };
FilterType currentFilter = KALMAN; // 默认使用卡尔曼滤波

// 滑动平均参数
#define SLIDING_WINDOW_SIZE 5
float slidingWindow[SLIDING_WINDOW_SIZE] = {0};
int slidingIndex = 0;
float slidingSum = 0.0;

// 限幅平均参数
#define LIMIT_THRESHOLD 5.0 // 最大允许速度变化 (km/h)

// 加权平均参数
#define WEIGHTED_WINDOW_SIZE 5
float weights[WEIGHTED_WINDOW_SIZE] = {0.1, 0.15, 0.2, 0.25, 0.3}; // 权重需和为1
float weightedWindow[WEIGHTED_WINDOW_SIZE] = {0};
int weightedIndex = 0;
float lastAvg = 0.0;

// 一阶低通滤波参数
float lowPassAlpha = 0.3; // 滤波系数 (0.1~0.5)
float lowPassFiltered = 0.0;

// 卡尔曼滤波参数
typedef struct {
  float q; // 过程噪声 (0.001~0.1)
  float r; // 观测噪声 (0.1~1)
  float p; // 估计误差协方差
  float x; // 估计值
} Kalman;
Kalman kalmanState = {0.1, 0.1, 1.0, 0.0};

// 界面状态机
enum DisplayState { MEASURING, SETTING_MENU, STATS, CONFIRM_RESET, ABOUT };
DisplayState displayState = MEASURING;

// 菜单相关变量
enum MenuItem { DIAMETER_SET, SPEED_SET, MAGNET_SET };
MenuItem selectedMenuItem = DIAMETER_SET;
uint8_t editPosition = 0;
// 编辑模式状态
struct EditState {
  bool isEditing = false;
  MenuItem currentItem;
  uint8_t cursorPos;
  char originalValue[6];
};
EditState editState;                          // 初始化结构

// 屏幕对象初始化
U8G2_ST7565_NHD_C12864_F_4W_SW_SPI u8g2(U8G2_R2, LCD_SCK, LCD_SDA, LCD_CS, LCD_DC, LCD_RST);

void setup() {
  // 初始化霍尔传感器
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(HALL_CONNECT_PIN, INPUT_PULLUP);

  // 使用Pico SDK处理中断
  gpio_set_irq_enabled_with_callback(HALL_SENSOR_PIN, GPIO_IRQ_EDGE_FALL, true, &hallSensorISR);
  
  // 初始化蜂鸣器引脚
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW); // 默认关闭

  // 初始化LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // 默认开启
  led.begin();            // 初始化LED
  led.setBrightness(25);  // 设置亮度（0-255）
  led.show();             // 初始关闭

  // 初始化按键
  int btnPins[] = {BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT, BTN_OK, BTN_BACK};
  for(int i=0; i<6; i++) pinMode(btnPins[i], INPUT_PULLUP);

  // 存储初始化
  EEPROM.begin(4096);
  // 从EEPROM读取数据
  loadConfig();

  // 初始化屏幕
  u8g2.begin();
  u8g2.setContrast(50);
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_tr);
  u8g2.drawUTF8(32,36,"Welcome");
  u8g2.setFont(u8g2_font_wqy13_t_gb2312);
  u8g2.drawUTF8(3,60,"Powered by Arduino");
  u8g2.sendBuffer();
  delay(2000);

  // 初始化时间基准
  lastUpdateTime = time_us_32() / 1000;
}

void loop() {
  // 检测霍尔传感器连接状态
  static bool lastHallState = HIGH;
  bool currentHallState = digitalRead(HALL_CONNECT_PIN);
  
  if (currentHallState != lastHallState) {
    hallCheckTime = millis();
    lastHallState = currentHallState;
  }
  
  if (millis() - hallCheckTime > hallWaitTime) {
    isHallConnected = !currentHallState; // 引脚拉低表示已连接
  }

  // 如果未连接，显示警告并跳过其他逻辑
  if (!isHallConnected) {
    u8g2.clearBuffer();
    u8g2.drawUTF8(8, 32, "霍尔传感器未连接!");
    u8g2.sendBuffer();
    updateLEDStatus(0);
    return;
  }

  // 处理脉冲计数
  noInterrupts(); // 禁用中断确保原子操作
  unsigned long currentPulses = pulseCount;
  pulseCount = 0; // 重置计数器
  interrupts();

  // 计算里程
  if (currentPulses > 0) {
	  float wheelCircum = config.wheelDiameter * 3.1416 / 1000.0;	// 周长（米）
	  float distancePerPulse = wheelCircum / config.magnetCount;	// 单次触发距离
	  totalDistanceFloat += distancePerPulse * currentPulses;		  // 浮点累积
    needsSave = true;
  }

  // 获取当前时刻
  unsigned long now = time_us_32() / 1000;
  // 速度计算逻辑
  if(now - lastUpdateTime >= 200){
    float rawSpeed = 0;
    // 停止检测
    if (lastTriggerTime != 0 && (now - lastTriggerTime) >= 2000) { // 2秒无信号视为停止
      lastTriggerTime = 0;
      pulseInterval = 0;
      rawSpeed = 0.0;
      resetAllFilters();
    }

    // 计算当前速度
    if (lastTriggerTime != 0 && pulseInterval > 0) {
      float wheelCircum = config.wheelDiameter * 3.1416 / 1000.0;
      rawSpeed = (wheelCircum * 3.6) / (pulseInterval / 1000.0 * config.magnetCount);
    } else {
      rawSpeed = 0.0;
    }

    // 应用滤波算法
    switch (currentFilter) {
      case SLIDING_AVG:
        currentSpeed = applySlidingAvg(rawSpeed);
        break;
      case LIMITED_AVG:
        currentSpeed = applyLimitedAvg(rawSpeed);
        break;
      case WEIGHTED_AVG:
        currentSpeed = applyWeightedAvg(rawSpeed);
        break;
      case LOW_PASS:
        currentSpeed = applyLowPass(rawSpeed);
        break;
      case KALMAN:
        currentSpeed = applyKalman(rawSpeed);
        break;
      default:
        currentSpeed = rawSpeed; // 默认不滤波
    }

    // 自动调节霍尔传感器消抖阀值
    if (currentSpeed > 20.0) {
      dynamicDebounce = 20;
    } else if (currentSpeed  > 5.0) {
      dynamicDebounce = 50;
    } else {
      dynamicDebounce = 100;
    }

    // 更新最大速度
    if(currentSpeed > config.maxSpeed) {
      config.maxSpeed = currentSpeed;
      needsSave = true;
    }

    // 蜂鸣器控制
    if (currentSpeed > config.overspeedThreshold) {
        digitalWrite(BUZZER, HIGH);
        isBuzzing = true;
    } else {
        digitalWrite(BUZZER, LOW);
        isBuzzing = false;
    }

    // 处理行驶计时
    if (currentSpeed > 0) {
        if (!isTraveling) {
            isTraveling = true;
            travelStartTime = now;
        }
    } else {
        if (isTraveling) {
            signleTravelTime += now - travelStartTime;
            deltaTravelTime = now - travelStartTime;
            isTraveling = false;
        }
    }

    // 数据保存
    if(currentSpeed == 0 && needsSave){
      totalTravelTimeFloat += deltaTravelTime / 1000.0;     // 毫秒转浮点秒
      deltaTravelTime = 0;
      saveConfig();
      needsSave = false;
    }

    lastUpdateTime = now;
  }

  // 按键扫描
  int btn = scanButtons();
  // 安全行驶功能：速度大于0时忽略按键并强制切换界面
  if (currentSpeed > 0.0) {
    displayState = MEASURING;     // 强制切换到测量界面
    if (editState.isEditing) {
      // 恢复原值
      switch(editState.currentItem) {
        case DIAMETER_SET:
          config.wheelDiameter = atoi(editState.originalValue);
          break;
        case SPEED_SET:
          config.overspeedThreshold = atof(editState.originalValue);
          break;
        case MAGNET_SET:
          config.magnetCount = atoi(editState.originalValue);
          break;
      }
      editState.isEditing = false;
    }
    btn = -1; // 忽略所有按键
  }

  // 更新LED状态
  updateLEDStatus(now);
  
  // 界面处理
  switch(displayState){
    case MEASURING:
      if (currentSpeed <= 0.0) {              // 仅当静止时响应按键
        if (btn == 4) {                       // 跳转设置界面
          displayState = SETTING_MENU;
          selectedMenuItem = DIAMETER_SET;
        } else if (btn == 1) {                // 清零单程时长
          signleTravelTime = 0;
        } else if (btn == 5) {                // 跳转统计界面
          displayState = STATS;
          confirmReset = false;
        }
      }
      drawMeasuring();
      break;
      
    case SETTING_MENU:
      handleSettingMenu(btn);                 // 设置界面按键处理交给函数处理
      drawSettingMenu();
      break;

    case STATS:
      if(btn == 4 && !confirmReset) {         // 进入关于界面
        displayState = ABOUT;
      } else if(btn == 1 && !confirmReset) {  // 进入确认清除对话
        confirmReset = true;
      } else if(confirmReset) {
        if(btn == 4) {                        // 确定清除
          totalDistanceFloat = 0;
          config.maxSpeed = 0;
          signleTravelTime = 0;
          totalTravelTimeFloat = 0;
          saveConfig();
          confirmReset = false;
        } else if(btn == 5) {                 // 取消清除
          confirmReset = false;
        }
      } else if(btn == 5) {                   // 退出统计界面
        displayState = MEASURING;
      }
      drawStats();
      break;

    case ABOUT:
      if(btn == 5) {                          // 返回统计界面
        displayState = STATS;
      }
      drawAbout();
      break;
  }
}

// 中断服务函数
void hallSensorISR(uint gpio, uint32_t events) {
  // 使用RP2040硬件定时器获取时间
  uint32_t currentTime = time_us_32() / 1000;
  // 未连接时禁用中断
  if (!isHallConnected) return;
  // 消抖处理：仅在间隔大于阈值时处理
  if (currentTime - lastTriggerTime >= dynamicDebounce) {
    // 首次触发时仅更新时间，不计数有效计数handleSettingMenu
    if (lastTriggerTime == 0) {
      lastTriggerTime = currentTime;
    } 
    // 后续正常处理
    else {
      pulseInterval = currentTime - lastTriggerTime;
      lastTriggerTime = currentTime;
      pulseCount++; // 有效计数
    }
  }
}

// 按键扫描函数
int scanButtons() {
  static unsigned long lastDebounceTime = 0;
  const uint8_t debounceDelay = 200;
  
  if(millis() - lastDebounceTime < debounceDelay) return -1;
  
  int btnMap[] = {BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT, BTN_OK, BTN_BACK};
  for(int i=0; i<6; i++){
    if(digitalRead(btnMap[i]) == LOW){
      lastDebounceTime = millis();
      return i;
    }
  }
  return -1;
}

// EEPROM操作
void saveConfig() {
  config.totalDistance = (unsigned long)totalDistanceFloat;     // 浮点转整数存储
  config.totalTravelTime = (unsigned long)totalTravelTimeFloat;
  EEPROM.put(0, config);
  EEPROM.commit();
  // 标记保存完成状态
  isBlinking = true;
  saveCompleteTime = millis();
}

void loadConfig() {
  EEPROM.get(0, config);
  // 检验数据是否合规
  if(config.wheelDiameter < 100 || config.wheelDiameter > 999){
    config.wheelDiameter = 700; // 默认700mm
  }
  if(config.overspeedThreshold < 10.0 || config.overspeedThreshold > 99.9){
    config.overspeedThreshold = 25.0; // 默认25km/h
  }
  if(config.magnetCount < 1 || config.magnetCount > 9){
    config.magnetCount = 1; // 默认1个磁铁
  }
  if(config.maxSpeed < 0 || config.maxSpeed > 50) {
    config.maxSpeed = 0;
  }
  if(config.totalTravelTime < 0) {
    config.totalTravelTime = 0;
  }
  totalDistanceFloat = config.totalDistance;		// 从整数转为浮点
  totalTravelTimeFloat = config.totalTravelTime;	// 从整数转为浮点
}

// 时间格式化
void formatTime(unsigned long milliseconds, char* buffer, size_t bufferSize, bool isTotal = false) {
  unsigned long totalSeconds = milliseconds / 1000;
  unsigned int hours = totalSeconds / 3600;
  unsigned int minutes = (totalSeconds % 3600) / 60;
  unsigned int seconds = totalSeconds % 60;

  if(isTotal) {
    snprintf(buffer, bufferSize, "%04u:%02u:%02u", hours, minutes, seconds); // 累计时长用4位小时
  } else {
    snprintf(buffer, bufferSize, "%02u:%02u:%02u", hours, minutes, seconds); // 单次时长保持2位小时
  }
}

// 参数编辑函数
void modifyValue(int8_t delta) {
  char buf[6];
  
  switch(selectedMenuItem){
    case DIAMETER_SET:
      snprintf(buf, sizeof(buf), "%03d", config.wheelDiameter);
      buf[editState.cursorPos] = (buf[editState.cursorPos] - '0' + delta + 10) % 10 + '0';
      config.wheelDiameter = constrain(atoi(buf), 100, 999);
      break;
      
    case SPEED_SET: {
      snprintf(buf, sizeof(buf), "%04.1f", config.overspeedThreshold);
      if(editState.cursorPos == 3) { // 小数位
        buf[4] = (buf[4] - '0' + delta + 10) % 10 + '0';
      } else { // 整数位
        buf[editState.cursorPos] = (buf[editState.cursorPos] - '0' + delta + 10) % 10 + '0';
      }
      float newVal = atof(buf);
      config.overspeedThreshold = constrain(newVal, 10.0, 99.9);
      break;
    }
    case MAGNET_SET: {
      config.magnetCount = constrain(config.magnetCount + delta, 1, 9);
      break;
    }
  }
}

// 界面绘制函数
void drawMeasuring() {
  u8g2.clearBuffer();
  
  // 显示速度
  char dispSpeed[5];
  sprintf(dispSpeed, "%04.1f", currentSpeed);   // 格式化速度
  u8g2.setCursor(0, 16);
  u8g2.print("速度");
  u8g2.setFont(u8g2_font_unifont_tr);           // 更改字体
  u8g2.drawUTF8(48,16,dispSpeed);
  u8g2.setFont(u8g2_font_wqy13_t_gb2312);       // 恢复字体
  u8g2.setCursor(96, 16);
  u8g2.print("km/h");

  // 显示里程
  char dispDistance[6];
  float currentDistance = totalDistanceFloat / 1000.0;   // 转换为km
//  float currentDistance = totalDistanceFloat;   // DEBUG时用m显示
  sprintf(dispDistance, "%08.1f", currentDistance);  // 格式化里程000000.0
  u8g2.setCursor(0, 32);
  u8g2.print("里程");
  u8g2.setFont(u8g2_font_unifont_tr);           // 更改字体
  u8g2.drawUTF8(32,32,dispDistance);
  u8g2.setFont(u8g2_font_wqy13_t_gb2312);       // 恢复字体
  u8g2.setCursor(105, 32);
  u8g2.print("km");

  // 超速警告
  if(currentSpeed > config.overspeedThreshold){
    u8g2.setCursor(8, 45);
    u8g2.print("已超速!注意减速！");
  }

  // 显示按键功能
  if(currentSpeed == 0){
    u8g2.setCursor(0, 62);
    u8g2.print("设置");
    u8g2.setCursor(102, 62);
    u8g2.print("统计");
  }

  // 单次行驶时间显示
  char timeBuffer[9];
  unsigned long currentTotal = signleTravelTime;
  if (isTraveling) {
    currentTotal += millis() - travelStartTime;
  }
  formatTime(currentTotal, timeBuffer, sizeof(timeBuffer), false);
  u8g2.setCursor(32, 62);
  u8g2.setFont(u8g2_font_unifont_tr);           // 更改字体
  u8g2.print(timeBuffer);
  u8g2.setFont(u8g2_font_wqy13_t_gb2312);       // 恢复字体

  u8g2.sendBuffer();
}

void drawSettingMenu() {
  u8g2.clearBuffer();
  
  // 绘制车轮直径项
  u8g2.setCursor(2, 12);
  u8g2.print("车轮直径");
  u8g2.setCursor(60, 12);
  u8g2.print(config.wheelDiameter);
  u8g2.setCursor(96, 12);
  u8g2.print("mm");
  
  // 绘制超速阈值项
  u8g2.setCursor(2, 28);
  u8g2.print("超速阈值");
  u8g2.setCursor(60, 28);
  u8g2.print(config.overspeedThreshold,1);
  u8g2.setCursor(96, 28);
  u8g2.print("km/h");

  // 新增磁铁数量项
  u8g2.setCursor(2, 44);
  u8g2.print("磁铁数量");
  u8g2.setCursor(60, 44);
  u8g2.print(config.magnetCount);

  // 绘制选择框
  // 调整选择框范围（新增第三项）
  int yPos = 0;
  switch(selectedMenuItem){
    case DIAMETER_SET: yPos = 0; break;
    case SPEED_SET:    yPos = 16; break;
    case MAGNET_SET:   yPos = 32; break;
  }
  u8g2.drawFrame(0, yPos, 128, 16);

  // 显示按键功能
  u8g2.setCursor(0, 62);
  u8g2.print("修改");
  u8g2.setCursor(102, 62);
  u8g2.print("退出");

  // 编辑模式指示
  if(editState.isEditing){
    int xStart = 0, yStart = 0;
    char buf[6];
    
    switch(selectedMenuItem){
      case DIAMETER_SET:
        dtostrf(config.wheelDiameter, 3, 0, buf);
        xStart = 60;
        yStart = 13;
        break;
      case SPEED_SET:
        dtostrf(config.overspeedThreshold, 4, 1, buf);
        xStart = 60;
        yStart = 29;
        break;
      case MAGNET_SET:
        dtostrf(config.overspeedThreshold, 1, 0, buf);
        xStart = 60;
        yStart = 45;
        break;
    }
    
    // 绘制数字位光标
    int digitWidth = (selectedMenuItem == DIAMETER_SET) ? 6 : 6;
    u8g2.drawHLine(xStart + editState.cursorPos*digitWidth, yStart, digitWidth);

    // 显示按键功能
    u8g2.drawBox(0, 50, 128, 14);     // 绘制白色框
    u8g2.setColorIndex(0);            // 设为白底黑字
    u8g2.setCursor(0, 62);
    u8g2.print("确认");
    u8g2.setCursor(102, 62);
    u8g2.print("取消");
    u8g2.setColorIndex(1);            // 恢复黑底白字
  }
  
  u8g2.sendBuffer();
}

void drawStats() {
  u8g2.clearBuffer();
  
  // 显示最大速度
  u8g2.setCursor(0, 15);
  u8g2.print("最大速度");
  u8g2.setCursor(60, 15);
  u8g2.print(config.maxSpeed, 1);
  u8g2.setCursor(96, 15);
  u8g2.print("km/h");

  // 显示平均速度
  float avgSpeed = 0;
  if(totalTravelTimeFloat > 0) {
    avgSpeed = (totalDistanceFloat / 1000.0) / (totalTravelTimeFloat / 3600.0); // 千米/小时
  }
  u8g2.setCursor(0, 30);
  u8g2.print("平均速度");
  u8g2.setCursor(60, 30);
  u8g2.print(avgSpeed, 1);
  u8g2.setCursor(96, 30);
  u8g2.print("km/h");

  // 显示累计时间
  char timeBuffer[13];
  formatTime(config.totalTravelTime * 1000, timeBuffer, sizeof(timeBuffer), true);
  u8g2.setCursor(0, 45);
  u8g2.print("累计时间");
  u8g2.setCursor(60, 45);
  u8g2.print(timeBuffer);

  // 显示按键功能
  u8g2.setCursor(0, 62);
  u8g2.print("关于");
  u8g2.setCursor(52, 62);
  u8g2.print("清除");
  u8g2.setCursor(102, 62);
  u8g2.print("退出");

  // 确认重置提示
  if(confirmReset) {
    u8g2.drawBox(0, 50, 128, 14);     // 绘制白色框
    u8g2.setColorIndex(0);            // 设为白底黑字
    u8g2.setCursor(32, 62);
    u8g2.print("确定清除？");
    u8g2.setCursor(0, 62);
    u8g2.print("是");
    u8g2.setCursor(115, 62);
    u8g2.print("否");
    u8g2.setColorIndex(1);            // 恢复黑底白字
  }

  u8g2.sendBuffer();
}

void drawAbout() {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_unifont_tr);
  u8g2.setCursor(20, 16);
  u8g2.print("Speedometer");
  u8g2.setCursor(6, 32);
  u8g2.print("Raspberry Pi Pico");
  u8g2.setCursor(0, 48);
  u8g2.print("Thanks for support");

  u8g2.setFont(u8g2_font_wqy13_t_gb2312);
  u8g2.setCursor(102, 62);
  u8g2.print("返回");
  
  u8g2.sendBuffer();
}

// 按键处理
void handleSettingMenu(int btn) {
  if(editState.isEditing){
    switch(btn){
      case 2: // LEFT
        if (editState.isEditing) {
          if (selectedMenuItem == SPEED_SET) {
            // 超速阈值特殊处理：0->3->1->0
            if (editState.cursorPos == 0) editState.cursorPos = 3;
            else if (editState.cursorPos == 3) editState.cursorPos = 1;
            else editState.cursorPos--;
          } else {
            editState.cursorPos = (editState.cursorPos - 1) % 3;
          }
        }
        break;

      case 3: // RIGHT
        if (editState.isEditing) {
          if (selectedMenuItem == SPEED_SET) {
            // 超速阈值特殊处理：0->1->3->0
            if (editState.cursorPos == 1) editState.cursorPos = 3;
            else if (editState.cursorPos == 3) editState.cursorPos = 0;
            else editState.cursorPos++;
          } else {
            editState.cursorPos = (editState.cursorPos + 1) % 3;
          }
        }
        break;
      case 0: // UP
        modifyValue(1);
        break;
      case 1: // DOWN
        modifyValue(-1);
        break;
      case 4: // OK - 确认修改
        editState.isEditing = false;
        needsSave = true;       // 标记需要保存
        saveConfig();           // 立即保存到EEPROM
        break;
      case 5: // BACK - 取消修改
        editState.isEditing = false;
        // 恢复原始值
        switch(editState.currentItem){
          case DIAMETER_SET:
            config.wheelDiameter = atoi(editState.originalValue);
            break;
          case SPEED_SET:
            config.overspeedThreshold = atof(editState.originalValue);
            break;
          case MAGNET_SET:
            config.magnetCount = atoi(editState.originalValue);
            break;
        }
        break;
    }
  } else {
    switch(btn){
      case 0: // UP
      case 1: // DOWN
        // 循环切换三个菜单项
        if(btn == 0) selectedMenuItem = (MenuItem)((selectedMenuItem + 2) % 3);
        if(btn == 1) selectedMenuItem = (MenuItem)((selectedMenuItem + 1) % 3);
        break;
      case 4: // OK
        if(!editState.isEditing) {  // 添加判断
          editState.isEditing = true;
          editState.currentItem = selectedMenuItem;
          editState.cursorPos = 0;
          // 保存原始值
          switch(selectedMenuItem){
            case DIAMETER_SET:
              dtostrf(config.wheelDiameter, 3, 0, editState.originalValue);
              break;
            case SPEED_SET:
              dtostrf(config.overspeedThreshold, 4, 1, editState.originalValue);
              break;
            case MAGNET_SET:
              snprintf(editState.originalValue, sizeof(editState.originalValue), "%d", config.magnetCount);
              break;
          }
        }
        break;
      case 5: // BACK
        displayState = MEASURING;
        break;
    }
  }
}

void updateLEDStatus(unsigned long now) {
  static unsigned long lastBlink = 0;
  const uint16_t blinkInterval = 200; // 闪烁间隔

  // 优先处理未连接状态
  if (!isHallConnected) {
    led.setPixelColor(0, led.Color(255, 0, 0));
    led.show();
    return;
  }

  // 处理保存完成后的状态
  if (isBlinking) {
    while (millis() - saveCompleteTime <= saveBlinkDuration) {
      // 蓝色
      if (millis() - lastBlink >= blinkInterval && isBlinking) {
        led.setPixelColor(0, led.Color(0, 0, 255));
        led.show();
        lastBlink = millis();
        isBlinking = false;
      } else if (millis() - lastBlink >= blinkInterval && !isBlinking) {
      // 熄灭
      led.setPixelColor(0, led.Color(0, 0, 0));
      led.show();
      lastBlink = millis();
      isBlinking = true;
      }
      continue;
    }
    isBlinking = false;
  }

  // 正常状态判断
  if (currentSpeed > config.overspeedThreshold) {
    // 超速：红色
    led.setPixelColor(0, led.Color(255, 0, 0));
    led.show();
  } else if (now - lastTriggerTime > 1000 && needsSave) {
    // 停车未保存：黄色
    led.setPixelColor(0, led.Color(255, 150, 0));
    led.show();
  } else if (currentSpeed > 0) {
    // 正常行驶：绿色
    led.setPixelColor(0, led.Color(0, 255, 0));
    led.show();
  } else {
    // 默认关闭
    led.setPixelColor(0, led.Color(0, 0, 0));
    led.show();
  }
}

// 滑动平均滤波
float applySlidingAvg(float speed) {
  slidingSum -= slidingWindow[slidingIndex];
  slidingWindow[slidingIndex] = speed;
  slidingSum += speed;
  slidingIndex = (slidingIndex + 1) % SLIDING_WINDOW_SIZE;
  return slidingSum / SLIDING_WINDOW_SIZE;
}

// 限幅平均滤波
float applyLimitedAvg(float speed) {
  float deltaSpeed = speed - lastAvg;
  if (abs(deltaSpeed) > LIMIT_THRESHOLD) {
    lastAvg = applySlidingAvg(speed + (deltaSpeed / abs(deltaSpeed)) * LIMIT_THRESHOLD);  // 限制加速度
    return lastAvg;
  } else {
    lastAvg = applySlidingAvg(speed); // 结合滑动平均
    return lastAvg;
  }
}

// 加权平均滤波
float applyWeightedAvg(float speed) {
  weightedWindow[weightedIndex] = speed;
  weightedIndex = (weightedIndex + 1) % WEIGHTED_WINDOW_SIZE;
  float sum = 0.0;
  for (int i=0; i<WEIGHTED_WINDOW_SIZE; i++) {
    sum += weightedWindow[i] * weights[i];
  }
  return sum;
}

// 一阶低通滤波
float applyLowPass(float speed) {
  lowPassFiltered = lowPassAlpha * speed + (1 - lowPassAlpha) * lowPassFiltered;
  return lowPassFiltered;
}

// 卡尔曼滤波
float applyKalman(float speed) {
  // 预测
  kalmanState.p += kalmanState.q;
  // 更新
  float k = kalmanState.p / (kalmanState.p + kalmanState.r);
  kalmanState.x += k * (speed - kalmanState.x);
  kalmanState.p *= (1 - k);
  return kalmanState.x;
}

// 重置滤波器
void resetAllFilters() {
  // 重置滑动平均滤波器
  for (int i = 0; i < SLIDING_WINDOW_SIZE; i++) {
    slidingWindow[i] = 0.0; // 清空窗口
  }
  slidingIndex = 0;
  slidingSum = 0.0;
  lastAvg = 0.0;

  // 重置加权平均滤波器
  for (int i = 0; i < WEIGHTED_WINDOW_SIZE; i++) {
    weightedWindow[i] = 0.0; // 清空窗口
  }
  weightedIndex = 0;

  // 重置低通滤波器
  lowPassFiltered = 0.0;

  // 重新初始化卡尔曼参数
  kalmanState.p = 1.0;   // 初始协方差
  kalmanState.x = 0.0;   // 初始估计值
}