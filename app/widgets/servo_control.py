"""
Servo control widget with increment/decrement buttons
"""
import time
from kivy.clock import Clock
from kivy.properties import NumericProperty
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.label import Label

from widgets.triangle_button import TriangleButton


class ServoControl(FloatLayout):
    """Servo control with increment/decrement triangle buttons"""
    MIN_CHANGE = 2
    min_value = NumericProperty(0)
    max_value = NumericProperty(110)
    value = NumericProperty(0)
    step_rate = NumericProperty(10.0)  # degrees per second

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Max label at top
        self.max_label = Label(
            text=f"{int(self.max_value)}°",
            size_hint=(1, 0.1),
            pos_hint={"center_x": 0.5, "top": 1},
            font_size="14sp",
            color=(0.7, 0.7, 0.7, 1),
        )
        self.add_widget(self.max_label)

        # Increment button (up triangle)
        self.inc_button = TriangleButton(
            direction="up",
            size_hint=(0.8, 0.25),
            pos_hint={"center_x": 0.5, "top": 0.9},
        )
        self.add_widget(self.inc_button)

        # Value label in center (between buttons)
        self.value_label = Label(
            text=f"{int(self.value)}°",
            size_hint=(1, 0.15),
            pos_hint={"center_x": 0.5, "center_y": 0.5},
            font_size="20sp",
            bold=True,
            color=(1, 1, 1, 1),
        )
        self.add_widget(self.value_label)

        # Decrement button (down triangle)
        self.dec_button = TriangleButton(
            direction="down",
            size_hint=(0.8, 0.25),
            pos_hint={"center_x": 0.5, "y": 0.1},
        )
        self.add_widget(self.dec_button)

        # Min label at bottom
        self.min_label = Label(
            text=f"{int(self.min_value)}°",
            size_hint=(1, 0.1),
            pos_hint={"center_x": 0.5, "y": 0},
            font_size="14sp",
            color=(0.7, 0.7, 0.7, 1),
        )
        self.add_widget(self.min_label)

        # Update loop
        self.last_update = time.time()
        self.last_sent_value = 0
        Clock.schedule_interval(self.update_value, 1 / 30)  # 30 FPS

    def update_value(self, dt):
        """Update servo value while buttons are pressed"""
        changed = False

        if self.inc_button.pressed:
            self.value += self.step_rate * dt
            if self.value > self.max_value:
                self.value = self.max_value
            changed = True

        if self.dec_button.pressed:
            self.value -= self.step_rate * dt
            if self.value < self.min_value:
                self.value = self.min_value
            changed = True

        if changed:
            self.value_label.text = f"{int(self.value)}°"

            # Send update every MIN_CHANGE degrees change
            if abs(self.value - self.last_sent_value) >= ServoControl.MIN_CHANGE or (
                self.value == 0 and self.last_sent_value != self.value):
                self.last_sent_value = self.value
                if hasattr(self, "on_value_change"):
                    print("ServoControl : Angle changed to -", self.value)
                    self.on_value_change(self.value)
