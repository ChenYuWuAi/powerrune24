# PowerRune 24
![正面介绍图](Front.jpg)
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
- 如果需要OTA升级，在开机前请配置OTA使用的Wifi，SSID配置为`3SE-120`，密码配置为`roboclub`，加密方式配置为`WPA2`，连接频段配置为`2.4GHz`，上电后，大符将连接Wifi，自动更新。

上电后大符将进行自动更新和初始化，完成后，大符R标将显示为待命状态，此时BLE进入可以连接状态。
### 2. 连接PowerRune Operator
- 打开PowerRune Operator。
- 软件会自动执行连接操作，连接成功后，软件会显示大符的状态信息。
- 如果连接失败，请检查大符电源是否正常，是否完成初始化，是否在可连接状态，是否正在更新。
- 如果初次连接失败，请尝试按C重新连接，或者重启大符电源。
- 如果多次连接失败，请联系技术支持。
### 3. 操作大符
- 选择相应的启动参数之后，点击`启动`按钮，大符将开始运行。
![输入图片说明](Instruction01.png)
- 大符运行过程中，可以通过`停止`按钮停止大符运行。
- 大符运行时产生的环数成绩会显示在软件界面上。
![输入图片说明](Instruction02.png)
- 大符运行时，可以手动观察大符的运行日志，分析击打结果。
![输入图片说明](Instruction03.png)
### 4. OTA升级、参数整定
该部分功能未在PowerRune Operator内开放。

如有需要，请联系技术支持。

支持调整的参数包括：
- PID参数
- 靶环亮度
- 靶臂亮度
- 流水灯亮度
- R标亮度
- 开机自动OTA升级开关
- OTA Wifi配置

# 已知问题
### 1. PowerRune Operator报错。
- 请检查是否已经安装了所需的库，包括textual和bleak。
- 尝试重启程序，因为蓝牙异常断开时会报错。
### 2. 大符靶子工作异常，似乎某些扇叶出现了重启，击打无反应，或者靶子无法停止。
- 请不要频繁击打，因为大符击打的中断过度激活会导致靶子重启。
- 靶子异常重启后，由于通信机制设置，防止数据不安全或者紊乱出现，用户需要手动切断电源，等待5秒后重新连接电源，等待靶子自动初始化。
- 请检查大符周围是否有干涉物，是否有异物进入靶子内部。
- 请检查大符电源是否正常，运行过程中是否有异常灯效、异常声音或者异常气味。如果有异常，请立即切断电源，不要重新连接电源，并立即联系技术支持。
### 3. Operator提示的方向与大符BLE提示的方向不一致。
- 请以Operator提示的方向为准，因为Operator是大符的控制中心，大符BLE提示的方向有误是开发阶段的遗留问题，不影响使用。