#ifndef MAIN_HPP
#define MAIN_HPP

// 屏幕引脚定义
#define LCD_SCK 2
#define LCD_SDA 3
#define LCD_CS 5
#define LCD_RST 6
#define LCD_DC 7

// 按键引脚定义
#define BTN_UP    20
#define BTN_DOWN  21
#define BTN_LEFT  22
#define BTN_RIGHT 23
#define BTN_OK    18
#define BTN_BACK  19

// 霍尔传感器引脚
#define HALL_SENSOR_PIN 27
#define HALL_CONNECT_PIN 26

// 蜂鸣器引脚定义
#define BUZZER 16

// LED引脚定义
// LED_BUILTIN 25
#define WS2812_PIN 24
#define WS2812_NUM 1

// 中断服务函数
void hallSensorISR(uint gpio, uint32_t events);

// 按键扫描函数
int scanButtons();

// EEPROM操作
void saveConfig();
void loadConfig();

// 时间格式化
void formatTime(unsigned long milliseconds, char* buffer, size_t bufferSize, bool isTotal);

// 参数编辑函数 
void modifyValue(int8_t delta);

// 界面绘制函数
void drawMeasuring();
void drawSettingMenu();
void drawStats();
void drawAbout();

// 按键处理
void handleSettingMenu(int btn);

// LED更新
void updateLEDStatus(unsigned long now);

// 速度平滑滤波
// 滑动平均滤波
float applySlidingAvg(float speed);
// 限幅平均滤波
float applyLimitedAvg(float speed);
// 加权平均滤波
float applyWeightedAvg(float speed);
// 一阶低通滤波
float applyLowPass(float speed);
// 卡尔曼滤波
float applyKalman(float speed);
// 重置滤波器
void resetAllFilters();

#endif