# PowerRune 24

# 介绍
基于ESP-IDF的24赛季大能量机关，机关各部件之间通过无线连接，支持完整的操作模式、方向和灯效调整功能，支持蓝牙参数整定、环数成绩获取、OTA升级。

# 快速使用入门
## PowerRune Operator部署
### 平台要求
- Windows或Linux
- 支持蓝牙BLE
### 环境配置
#### 1. 安装Python 3.7以上版本，并配置python、pip的环境变量。
基础操作，自行上网搜索即可。
#### 2. 执行安装脚本。
```shell
git clone https://gitee.com/chenyuwuai/powerrune24.git
```
```shell
cd ./powerrune24/PowerRune24-Operator/
```
```shell
./install.sh
```
- Windows用户请使用`install.bat`。
- Linux用户请使用`install.sh`。
- 本脚本会自动安装所需的Textual和Bleak库。
#### 3. 运行PowerRune Operator。
```shell
python pr-24-operator.py
```
## 快速使用
### 1. 连接大符电源
确保大符电源已经开启。
- 将大符移动到目标位置。
- 确认大符周围净空，防止运转中干涉。
- 将大符`220V`电源插头插入电源插座。
- 确认大符电源指示灯亮起，或听到风扇正常运转声音。

此时，大符将进行自动更新和初始化，完成后，大符R标将显示为待命状态，此时BLE进入可以连接状态。
### 2. 连接PowerRune Operator
- 打开PowerRune Operator。
- 软件会自动执行连接操作，连接成功后，软件会显示大符的状态信息。
- 如果连接失败，请检查大符电源是否正常，是否完成初始化，是否在可连接状态，是否正在更新。
- 如果初次连接失败，请尝试按C重新连接，或者重启大符电源。
- 如果多次连接失败，请联系技术支持。
### 3. 操作大符
- 选择相应的启动参数之后，点击`启动`按钮，大符将开始运行。
- 大符运行过程中，可以通过`停止`按钮停止大符运行。
- 大符运行时产生的环数成绩会显示在软件界面上。
- 大符运行时，可以手动观察大符的运行日志，分析击打结果。
### 4. OTA升级、参数整定
该部分功能未在PowerRune Operator内开放。

如有需要，请联系技术支持。

支持调整的参数包括：
- PID参数
- 靶环亮度
- 靶臂亮度
- 流水灯亮度
- R标亮度
- 开机自动OTA升级
