# PicoSpeedometer

PicoSpeedometer是一款基于RP2040的自行车速度和里程记录表，作为我的本科毕业设计。

---

## 功能

- 速度和里程测量
- 自行车参数修改
- 累计行驶数据统计

## 如何使用

本项目使用[PlatformIO](https://platformio.org/)和[Arduino-Pico](https://github.com/earlephilhower/arduino-pico)开发，在PlatformIO IDE中打开本项目，将会自动部署项目环境。部署完成连接RP2040开发板编译上传程序到单片机即可。对了，你要自己购买相关的外设模块。

---

## 作者说明

这个项目很烂，能完成毕业要求就够了，有机会再改进吧

---

## 画饼

- [ ] FreeRTOS重构
- [ ] 更详细的参数设置
- [ ] 更丰富的显示
- [ ] GPS定位

## Thanks

- [PlatformIO](https://platformio.org/)
- [Arduino-Pico](https://github.com/earlephilhower/arduino-pico)
- [u8g2](https://github.com/olikraus/u8g2)
- [Adafruit NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel)