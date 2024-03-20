from textual import on
import asyncio
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

version = "v1.0.0"
connected = False
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

    def compose(self) -> ComposeResult:
        with Horizontal(id="operation_buttons"):
            yield Button("▶启动", id="start", variant="success")
            yield Button("↻停止", id="stop", variant="warning")
        # 得分
        with Collapsible(title="得分", collapsed=False):
            yield DataTable()
        with Collapsible(title="启动参数", collapsed=False):
            with Horizontal(id="start_params"):
                yield Label("颜色方")
                with RadioSet():
                    yield RadioButton("红方", value=True)
                    yield RadioButton("蓝方")
                yield Label("启动模式")
                with RadioSet():
                    yield RadioButton("大符模式", value=True)
                    yield RadioButton("小符模式")
                yield Label("循环")
                yield Switch(value=True)

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """An action to start the PowerRune."""
        if event.button.id == "start":
            if connected:
                self.notify("正在发送启动参数...", title="提示", severity="information")
                log.write_line("[Info] 正在发送启动参数，颜色方：")
                # css .started
                self.add_class("started")
            else:
                self.notify("设备未连接", title="错误", severity="error")
                log.write_line("[Error] 设备未连接")
        elif event.button.id == "stop":
            if connected:
                self.notify("正在发送停止指令...", title="提示", severity="information")
                log.write_line("[Info] 正在发送停止指令...")
                # css .started
                self.remove_class("started")
            else:
                self.notify("设备未连接", title="错误", severity="error")
                log.write_line("[Error] 设备未连接")

    def on_mount(self) -> None:
        table = self.query_one(DataTable)
        table.add_columns(*("颜色", "模式", "得分", "时间"))


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
            with TabPane("系统设置", id="settings"):
                yield ScrollableContainer(PowerRune24_Settings())
        yield Footer()

    def action_toggle_dark(self) -> None:
        """An action to toggle dark mode."""
        self.dark = not self.dark

    def action_connect(self) -> None:
        """An action to connect to the device."""
        global connected
        connected = not connected
        if connected:
            self.sub_title = "已连接 - " + version
        else:
            self.sub_title = "未连接 - " + version

    def action_quit(self) -> None:
        """An action to quit the app."""
        exit()

    def on_mount(self) -> None:
        self.title = "PowerRune24 控制面板"
        self.sub_title = "未连接 - " + version
        self.notify(
            "欢迎使用 PowerRune24 控制面板。尝试自动连接设备中...",
            title="提示",
            severity="information",
        )
        global log
        log = self.query_one(Log)
        log.write_line("[Info] 欢迎使用 PowerRune24 控制面板。尝试自动连接设备中...")


if __name__ == "__main__":
    app = PowerRune24_Operator()
    app.run()
