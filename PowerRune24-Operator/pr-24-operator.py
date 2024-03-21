import re
import time
import asyncio
import threading
from bleak import BleakScanner, BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic
from textual.app import App, ComposeResult, Notify
from textual.containers import ScrollableContainer, Horizontal
from textual.widgets import (
    Select,
    DataTable,
    Log,
    Input,
    Collapsible,
    Header,
    Footer,
    Switch,
    Button,
    Static,
    Label,
    RadioButton,
    RadioSet,
    TabbedContent,
    TabPane,
    LoadingIndicator,
)

# 显式定义字符串
DEVICE_NAME = "PowerRune24"
version = "v1.0.0"
UUID_Serv_Config = "00001827-0000-1000-8000-00805f9b34fb"
UUID_Serv_Operation = "00001828-0000-1000-8000-00805f9b34fb"
UUID_Char_URL = "00002aa6-0000-1000-8000-00805f9b34fb"
UUID_Char_SSID = "00002ac3-0000-1000-8000-00805f9b34fb"
UUID_Char_PSK = "00002a3e-0000-1000-8000-00805f9b34fb"
UUID_Char_AutoOTA = "00002ac5-0000-1000-8000-00805f9b34fb"
UUID_Char_Brightness_Armour = "00002a0d-0000-1000-8000-00805f9b34fb"
UUID_Char_Brightness_Arm = "00002a01-0000-1000-8000-00805f9b34fb"
UUID_Char_Brightness_RLogo = "00002a9b-0000-1000-8000-00805f9b34fb"
UUID_Char_Brightness_Matrix = "00002a9c-0000-1000-8000-00805f9b34fb"
UUID_Char_PID = "00002a66-0000-1000-8000-00805f9b34fb"
UUID_Char_Reset_Armour_ID = "00002b1f-0000-1000-8000-00805f9b34fb"
UUID_Char_RUN = "00002a65-0000-1000-8000-00805f9b34fb"
UUID_Char_Score = "00002a69-0000-1000-8000-00805f9b34fb"
UUID_Char_Unlock = "00002a3b-0000-1000-8000-00805f9b34fb"
UUID_Char_Stop = "00002ac8-0000-1000-8000-00805f9b34fb"
UUID_Char_OTA = "00002a9f-0000-1000-8000-00805f9b34fb"


class PowerRune24_Operations(Static):
    """A widget to display the available operations and attributes of PowerRune."""

    global client

    def compose(self) -> ComposeResult:
        with Horizontal(id="operation_buttons"):
            yield Button("▶启动", id="start", variant="success")
            yield Button("↻停止", id="stop", variant="warning")
            yield Label("", id="state")
        # 得分
        with Collapsible(title="得分", collapsed=False):
            yield DataTable(id="score")
        with Collapsible(title="启动参数", collapsed=False):
            with Horizontal(id="start_params"):
                yield Label("颜色方")
                with RadioSet(id="color"):
                    yield RadioButton("红方", value=True)
                    yield RadioButton("蓝方")
                yield Label("启动模式")
                with RadioSet(id="mode"):
                    yield RadioButton("大符模式", value=True)
                    yield RadioButton("小符模式")
                yield Label("循环")
                yield Switch(value=True, id="loop")
                yield Label("方向")
                with RadioSet(id="direction"):
                    yield RadioButton("顺时针", value=True)
                    yield RadioButton("逆时针")

    def BLE_notify_handler(self, sender, data):
        # byte array->string
        data = data.replace(b"\x00", b"").decode("utf-8")
        # 更新log
        log.write_line("[BLE] %s" % data)
        self.notify(data, title="大符", severity="information")
        if (data == "PowerRune Activation Failed" and self.has_class("started")):
            self.update_score(0)
        # 正则表达式提取，目标为"[Score: %d]PowerRune Activated Successfully", score，不定长
        if re.match(r"\[Score: \d+\]PowerRune Activated Successfully", data):
            score = int(re.findall(r"\d+", data)[0])
            self.update_score(score)
        if (data == "PowerRune Run Complete"):
            self.remove_class("started")       
        
    async def on_button_pressed(self, event: Button.Pressed) -> None:
        """An action to start the PowerRune."""
        if event.button.id == "start":
            if connected:
                if not self.first_notify_enabled:
                    self.first_notify_enabled = True
                    await client.start_notify(UUID_Char_RUN, self.BLE_notify_handler)
                self.notify("正在发送启动参数...", title="提示", severity="information")
                self.start_params = (
                    self.query_one("#color").pressed_index,
                    self.query_one("#mode").pressed_index,
                    self.query_one("#loop").value,
                    self.query_one("#direction").pressed_index,
                )
                self.query_one("#state").update(
                    "  %s %s %s %s  "
                    % (
                        (
                            "颜色:red_circle:"
                            if not self.start_params[0]
                            else "颜色:blue_circle:"
                        ),
                        "大符模式" if not self.start_params[1] else "小符模式",
                        "循环✓" if self.start_params[2] else "循环✕",
                        "方向↻" if not self.start_params[3] else "方向↺",
                    )
                )
                log.write_line(
                    "[Info] 正在发送启动参数，颜色方：%s，启动模式：%s，循环：%s，方向：%s"
                    % (
                        "红方" if not self.start_params[0] else "蓝方",
                        "大符模式" if not self.start_params[1] else "小符模式",
                        "是" if self.start_params[2] else "否",
                        "顺时针" if not self.start_params[3] else "逆时针",
                    )
                )
                # send to characteristic
                await client.write_gatt_char(
                    UUID_Char_RUN,
                    bytes(
                        [
                            self.start_params[0],
                            self.start_params[1],
                            self.start_params[2],
                            self.start_params[3],
                        ]
                    ),
                )
                # css .started
                self.add_class("started")
            else:
                self.notify("设备未连接", title="错误", severity="error")
                log.write_line("[Error] 设备未连接")
        elif event.button.id == "stop":
            if connected:
                self.notify("正在发送停止指令...", title="提示", severity="information")
                # send to characteristic
                await client.write_gatt_char(UUID_Char_Stop, bytes([0]))
                log.write_line("[Info] 正在发送停止指令...")
                # css .started
                self.remove_class("started")
            else:
                self.notify("设备未连接", title="错误", severity="error")
                log.write_line("[Error] 设备未连接")

    async def on_mount(self) -> None:
        self.first_notify_enabled = False
        table = self.query_one(DataTable)
        table.add_columns(*("颜色", "模式", "得分", "时间"))

    def update_score(self, score: int) -> None:
        table = self.query_one(DataTable)
        table.add_row(
            "红方" if not self.start_params[0] else "蓝方",
            "大符" if not self.start_params[1] else "小符",
            score,
            time.strftime("%H:%M:%S", time.localtime()),
        )


class PowerRune24_Settings(Static):
    """A widget to display the available settings of PowerRune."""

    def compose(self) -> ComposeResult:
        # 保存按钮
        with ScrollableContainer():
            yield Button("保存", id="save", variant="success")
            # yield LoadingIndicator()
            with Collapsible(title="网络和更新设置", collapsed=True):
                yield Label("更新服务器URL")
                yield Input(placeholder="请输入URL", id="url")

                yield Label("SSID")
                yield Input(placeholder="请输入SSID", id="ssid")

                yield Label("密码")
                yield Input(placeholder="请输入密码", id="psk")

                yield Label("自动OTA")
                yield Switch(value=True, id="auto_ota")
            with Collapsible(title="亮度设置", collapsed=True):

                yield Label("大符环数靶亮度")
                yield Select(((str(i), str(i)) for i in range(0, 256)), id="brightness")

                yield Label("大符臂亮度")
                yield Select(
                    ((str(i), str(i)) for i in range(0, 256)), id="brightness_arm"
                )

                yield Label("R标亮度")
                yield Select(
                    ((str(i), str(i)) for i in range(0, 256)), id="brightness_rlogo"
                )

                yield Label("点阵亮度")
                yield Select(
                    ((str(i), str(i)) for i in range(0, 256)), id="brightness_matrix"
                )
            with Collapsible(title="PID设置", collapsed=True):

                yield Label("kP值")
                yield Input(placeholder="请输入kP值", id="kp")
                yield Label("kI值")
                yield Input(placeholder="请输入kI值", id="ki")
                yield Label("kD值")
                yield Input(placeholder="请输入kD值", id="kd")
                # i_max, d_max, o_max

                yield Label("i_max值")
                yield Input(placeholder="请输入i_max值", id="i_max")
                yield Label("d_max值")
                yield Input(placeholder="请输入d_max值", id="d_max")
                yield Label("o_max值")
                yield Input(placeholder="请输入o_max值", id="o_max")

            with Collapsible(title="高级操作", collapsed=True):
                with Horizontal(id="advanced_buttons"):
                    yield Button("OTA", id="ota", variant="primary")
                    yield Button("重置装甲板ID", id="reset", variant="warning")


class PowerRune24_Operator(App):
    """A Textual app to manage stopwatches."""

    ENABLE_COMMAND_PALETTE = False
    CSS_PATH = "pr-24-operator.tcss"
    BINDINGS = [
        ("d", "toggle_dark", "颜色模式切换"),
        ("q", "quit", "退出"),
        ("c", "connect", "连接设备"),
    ]

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        yield Header(show_clock=True)
        with TabbedContent(initial="operations"):
            with TabPane("操作", id="operations"):  # First tab
                yield ScrollableContainer(PowerRune24_Operations())
            with TabPane("日志", id="logs"):
                yield Log(id="log")
            # with TabPane("系统设置", id="settings"):
            #     yield ScrollableContainer(PowerRune24_Settings())
        yield Footer()

    def action_toggle_dark(self) -> None:
        """An action to toggle dark mode."""
        self.dark = not self.dark

    async def action_connect(self) -> None:
        """An action to connect to the device."""
        if connected:
            self.notify("设备已连接", title="提示", severity="information")
        elif self.connecting:
            self.notify("设备正在连接", title="注意", severity="warning")
        else:
            self.run_thread_loop(self.connect_device())

    def action_quit(self) -> None:
        """An action to quit the app."""
        if connected:
            client.disconnect()
        self.exit()

    def run_thread_loop(self, task):
        def thread_loop_task(loop, task):

            # 为子线程设置自己的事件循环
            asyncio.set_event_loop(loop)
            loop.run_until_complete(task)

        t = threading.Thread(target=thread_loop_task, args=(self.thread_loop, task))
        t.daemon = True
        t.start()

    def on_mount(self) -> None:
        """A lifecycle hook that runs when the app mounts."""
        global connected, client
        self.devices = []
        self.device = None
        self.services = []
        self.characteristics = []
        self.descriptors = []
        self.service = None
        self.characteristic = None
        self.loop = asyncio.get_event_loop()
        client = None
        self.title = "PowerRune24 控制面板"
        connected = False
        self.sub_title = "未连接 - " + version

        global log
        log = self.query_one(Log)
        self.notify(
            "欢迎使用 PowerRune24 控制面板。",
            title="提示",
            severity="information",
        )
        log.write_line("[Info] 欢迎使用 PowerRune24 控制面板。")
        # 创建一个事件循环thread_loop
        self.thread_loop = asyncio.new_event_loop()
        # 将thread_loop作为参数传递给子线程
        self.run_thread_loop(self.connect_device())

    async def connect_device(self):
        self.connecting = True
        global connected, client
        # BLE client
        # Name: PowerRune24
        self.notify("正在搜索设备...", title="提示", severity="information")
        log.write_line("[Info] 正在搜索设备...")
        try:
            self.device = await BleakScanner.find_device_by_filter(
                lambda d, ad: d.name and d.name.lower() == DEVICE_NAME.lower()
            )
        except Exception as e:
            if "object has no attribute" in str(e):
                self.notify("未找到设备", title="错误", severity="error")
                log.write_line("[Error] 未找到设备")
                self.connecting = False
                return
            self.notify(str(e), title="错误", severity="error")
            log.write_line("[Error] " + str(e))
            self.connecting = False
            return

        client = BleakClient(self.device, disconnected_callback=self.on_disconnect)
        try:
            await client.connect()
        except Exception as e:
            if "object has no attribute" in str(e):
                self.notify("未找到设备", title="错误", severity="error")
                log.write_line("[Error] 未找到设备")
                self.connecting = False
                return
            self.notify(str(e), title="错误", severity="error")
            log.write_line("[Error] " + str(e))
            try:
                await client.disconnect()
            except:
                self.notify(str(e), title="错误", severity="error")
                log.write_line("[Error] " + str(e))
            client = None
            self.connecting = False
            return

        if client.is_connected:
            self.notify(
                "成功连接到大符设备 %s" % self.device.address,
                title="提示",
                severity="information",
            )
            # Check the appearance in the generic attribute. If it's not 0x09F0, it means it's an unknown device, not PowerRune.
            # In this case, clear the client, select a device again, and raise an error.
            connected = True
            self.sub_title = "已连接 - " + version
            log.write_line("[Info] 成功连接到大符设备 %s" % self.device.address)
            await asyncio.sleep(1)
        else:
            self.notify(
                "尝试连接大符设备 %s 失败" % self.device.address,
                title="错误",
                severity="error",
            )
            log.write_line("[Error] 尝试连接大符设备 %s 失败" % self.device.address)
            connected = False
            self.sub_title = "未连接 - " + version
        self.connecting = False
        return

    def on_disconnect(self):
        # npyscreen notify_confirm
        global connected, client
        self.notify("设备已断开", title="提示", severity="information")
        log.write_line("[Info] 设备已断开")
        connected = False
        self.sub_title = "未连接 - " + version
        client = None
        if self.device:
            self.device = None
        # css .started
        self.remove_class("started")
        # css .reconnect
        self.add_class("reconnect")


if __name__ == "__main__":
    app = PowerRune24_Operator()
    app.run()
