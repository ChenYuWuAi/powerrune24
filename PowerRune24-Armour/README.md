# PowerRune24-Armour

## 项目简介

PowerRune24-Armour 是大能量机关项目中三大控制器之一，用于控制装甲板的状态。

## 功能特性

- 使用控制器：ESP32 S3
- 支持两种装甲板模式：小能量机关模式和大能量机关模式
- 移植了大符通用库：LED，LED_Strip，PowerRune_Event，PowerRune_Messenger，firmware库

## 使用方法

1. 硬件连接
2. ESP-IDF 环境搭建
3. 编译menuconfig，可以配置PowerRune_Configuration配置
4. 编译后推送到pr_ota_bin项目，进行OTA升级，或者USB-JTAG下载