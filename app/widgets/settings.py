"""
Settings widget for PID and WiFi configuration
"""
from kivy.properties import BooleanProperty, DictProperty, ListProperty, StringProperty
from kivy.storage.jsonstore import JsonStore
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.dropdown import DropDown
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput


class SettingsWidget(BoxLayout):
    options = ListProperty([])
    config_values = DictProperty({})
    store_path = StringProperty("config.json")
    is_expanded = BooleanProperty(False)

    def __init__(self, **kwargs):
        super().__init__(orientation="vertical", **kwargs)

        self.store = JsonStore(self.store_path)
        self.load_config()

        self.main_button = Button(
            text="Configurations",
            size_hint=(1, 1),
            background_color=(0.2, 0.2, 0.3, 1),
            on_press=self.toggle_dropdown,
        )
        self.add_widget(self.main_button)

        self.dropdown = DropDown(auto_dismiss=True)
        self.build_dropdown()

    def load_config(self):
        if self.store.exists("settings"):
            self.config_values = self.store.get("settings")
        else:
            defaults: dict[str, float | str] = {
                opt: 0.0 for opt in self.options if opt not in ["ssid", "passwd"]
            }
            defaults["kp"] = 1.0
            defaults["ssid"] = "NPhone"
            defaults["passwd"] = "wifi@2.4"
            self.config_values = defaults
            self.save_config()

    def save_config(self):
        self.store.put("settings", **self.config_values)

    def build_dropdown(self):
        self.dropdown.clear_widgets()

        for key in self.options:
            row = BoxLayout(orientation="horizontal", size_hint_y=None, height=100)
            label = Label(
                text=f"{key}: ",
                size_hint_x=0.4,
                color=(1, 1, 1, 1),
                bold=True,
            )

            if key in ["ssid", "passwd"]:
                value_input = TextInput(
                    text=str(self.config_values.get(key, "")),
                    multiline=False,
                    password=(key == "passwd"),
                    size_hint_x=0.6,
                    foreground_color=(1, 1, 1, 1),
                    background_color=(0.2, 0.2, 0.3, 1),
                )
            else:
                value_input = TextInput(
                    text=str(self.config_values.get(key, 0.0)),
                    multiline=False,
                    input_filter="float",
                    size_hint_x=0.6,
                    foreground_color=(1, 1, 1, 1),
                    background_color=(0.2, 0.2, 0.3, 1),
                )

            value_input.bind(text=lambda _, v, k=key: self.update_config(k, v))
            row.add_widget(label)
            row.add_widget(value_input)
            self.dropdown.add_widget(row)

        provision_btn = Button(
            text="Provision",
            size_hint_y=None,
            height=100,
            background_normal="",
            background_color=(0.3, 0.6, 0.3, 1),
        )
        provision_btn.bind(on_release=self.on_provision)
        self.dropdown.add_widget(provision_btn)

    def toggle_dropdown(self, *_):
        if not self.is_expanded:
            self.dropdown.open(self.main_button)
            self.is_expanded = True
        else:
            self.dropdown.dismiss()
            self.is_expanded = False
        self.build_dropdown()

    def update_config(self, key, value):
        try:
            if key in ["ssid", "passwd"]:
                self.config_values[key] = value
            else:
                self.config_values[key] = float(value) if value else 0.0
            self.save_config()
        except ValueError:
            pass

    def on_provision(self, instance):
        if hasattr(self, "provision_callback"):
            self.provision_callback()
