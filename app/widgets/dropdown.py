"""
Generic dropdown widget
"""

from kivy.properties import ListProperty, StringProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.dropdown import DropDown


class DropdownWidget(BoxLayout):
    header_text = StringProperty("")
    options = ListProperty([])
    value = StringProperty("")

    def __init__(self, **kwargs):
        super().__init__(orientation="vertical", **kwargs)

        self.header = Button(
            text=self.header_text or "Select Option",
            size_hint=(1, 1),
            background_normal="",
            background_color=(0.2, 0.2, 0.3, 1),
        )
        self.add_widget(self.header)

        self.dropdown = DropDown()
        self.populate_dropdown()
        self.header.bind(on_release=self.dropdown.open)
        self.dropdown.bind(on_select=self.on_select_option)
        self.bind(options=lambda *_: self.populate_dropdown())

    def populate_dropdown(self):
        self.dropdown.clear_widgets()
        for option in self.options:
            btn = Button(
                text=str(option),
                size_hint_y=None,
                height=100,
                background_normal="",
                background_color=(0.25, 0.25, 0.35, 1),
            )
            btn.bind(on_release=lambda btn, opt=option: self.dropdown.select(opt))
            self.dropdown.add_widget(btn)

    def on_select_option(self, instance, value):
        if hasattr(self, "on_select"):
            self.on_select(self, value)
