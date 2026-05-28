"""
Main application entry point
"""
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.image import Image
import time
import asyncio

from widgets.joystick import JoystickWidget
from widgets.servo_control import ServoControl
from widgets.dropdown import DropdownWidget
from widgets.settings import SettingsWidget
from network.server import MultiPortServer


class GyroControllerApp(App):
    """Main application"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.current_mode = "Object detection"
        self.current_info = "Joystick"
        self.server = MultiPortServer(self)
        self._last_control_send = 0
        self._control_interval = 0.05  # 20 Hz
        self._last_servo_value = 0
        self.esp32_ip = None  # Will be auto-detected

    def build(self):
        root = BoxLayout(orientation="vertical", padding=[0, 80, 0, 0])

        top_bar = BoxLayout(
            orientation="horizontal", size_hint=(1, 0.1), spacing=5, padding=5
        )
        bottom_area = FloatLayout(size_hint=(1, 0.9))

        # Mode dropdown
        self.mode_dropdown = DropdownWidget(
            header_text="Mode : Object detection",
            options=["Object detection", "2D Path Mapping"],
            size_hint=(0.33, 1),
        )
        self.mode_dropdown.on_select = self.on_mode_select
        top_bar.add_widget(self.mode_dropdown)

        # Info dropdown
        self.info_dropdown = DropdownWidget(
            header_text="Joystick : (0.0, 0.0)",
            options=["Joystick", "IMU", "Distance"],
            size_hint=(0.33, 1),
        )
        self.info_dropdown.on_select = self.on_info_select
        top_bar.add_widget(self.info_dropdown)

        # Settings dropdown
        self.settings_dropdown = SettingsWidget(
            options=["kp", "kd", "ki", "ssid", "passwd"],
            size_hint=(0.33, 1),
        )
        self.settings_dropdown.provision_callback = self.on_provision
        top_bar.add_widget(self.settings_dropdown)

        root.add_widget(top_bar)

        # Single camera display (processed image only)
        self.proc_image = Image(
            color=(0, 0, 0, 1), 
            size_hint=(1, 1), 
            pos_hint={"x": 0, "y": 0}
        )
        bottom_area.add_widget(self.proc_image)

        # Joystick
        self.joystick = JoystickWidget(
            size_hint=(None, None), size=(300, 300), pos_hint={"x": 0.08, "y": 0.08}
        )
        self.joystick.bind(angle=self.send_control, magnitude=self.send_control)
        bottom_area.add_widget(self.joystick)

        # Servo control with triangle buttons
        self.servo_control = ServoControl(
            min_value=0,
            max_value=110,
            value=0,
            step_rate=15.0,  # 15 degrees per second
            size_hint=(0.12, 0.6),
            pos_hint={"x": 0.86, "center_y": 0.5},
        )
        self.servo_control.on_value_change = self.on_servo_change
        bottom_area.add_widget(self.servo_control)

        root.add_widget(bottom_area)

        # Start servers
        self.server.start_all()
        print("[APP] Servers starting...")

        return root

    def on_mode_select(self, instance, mode):
        """Handle mode selection and send to ESP32"""
        self.current_mode = mode
        instance.header.text = f"Mode : {mode}"
        
        # Send mode change to ESP32 via UDP
        if self.server.server_ready and self.server.loop is not None:
            try:
                asyncio.run_coroutine_threadsafe(
                    self.server.send_mode_udp(mode),
                    self.server.loop,
                )
                print(f"[APP] Mode changed to: {mode}")
            except Exception as e:
                print(f"[APP] Error sending mode: {e}")

    def on_info_select(self, instance, info):
        """Handle info display selection"""
        self.current_info = info
        if info == "Joystick":
            instance.header.text = (
                f"{info} : ({self.joystick.magnitude:.2f}, {self.joystick.angle:.2f})"
            )
        else:
            instance.header.text = f"{info} : 0.0"

    def on_servo_change(self, value):
        """Called when servo value changes by 5 degrees"""
        print(f"[APP] Servo changed to {value:.1f}°")
        self.send_control()

    def send_control(self, *_):
        """Send control data via UDP to ESP32"""
        if not self.server.server_ready or self.server.loop is None:
            return

        angle = float(self.joystick.angle)
        magnitude = float(self.joystick.magnitude)
        servo = float(self.servo_control.value)

        # Update joystick display
        if self.current_info == "Joystick":
            self.info_dropdown.header.text = (
                f"Joystick : ({magnitude:.2f}, {angle:.1f}°)"
            )

        now = time.time()

        is_centered = magnitude == 0.0 and angle == 0.0
        should_send = is_centered or (
            now - self._last_control_send >= self._control_interval
        )

        if should_send:
            self._last_control_send = now
            try:
                asyncio.run_coroutine_threadsafe(
                    self.server.send_control_udp(magnitude, angle, servo),
                    self.server.loop,
                )
            except Exception as e:
                print(f"[APP] Error sending control: {e}")

    def on_provision(self):
        """Handle WiFi provisioning - send PID + credentials via TCP to ESP32"""
        if not self.server.server_ready or self.server.loop is None:
            print("[APP] Server not ready yet")
            return

        if self.esp32_ip is None:
            print("[APP] ESP32 IP not detected. Wait for IMU data first.")
            return

        config = self.settings_dropdown.config_values

        kp = float(config.get("kp", 1.0))
        ki = float(config.get("ki", 0.0))
        kd = float(config.get("kd", 0.0))
        ssid = str(config.get("ssid", ""))
        passwd = str(config.get("passwd", ""))

        print(f"[APP] Provisioning ESP32 @ {self.esp32_ip}")
        print(f"[APP] PID: kp={kp}, ki={ki}, kd={kd}")
        print(f"[APP] WiFi: SSID='{ssid}'")

        try:
            asyncio.run_coroutine_threadsafe(
                self.server.send_settings_tcp(kp, ki, kd, ssid, passwd),
                self.server.loop,
            )
        except Exception as e:
            print(f"[APP] Error provisioning: {e}")


if __name__ == "__main__":
    GyroControllerApp().run()
