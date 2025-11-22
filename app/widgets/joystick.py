"""
Joystick widget for controlling robot movement
"""
import math
from kivy.clock import Clock
from kivy.graphics import Canvas, Color, Ellipse, Line
from kivy.properties import NumericProperty
from kivy.uix.widget import Widget


class JoystickWidget(Widget):
    angle = NumericProperty(0)
    magnitude = NumericProperty(0)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.knob_x = None
        self.knob_y = None
        self.user_has_touched = False
        if self.canvas is None:
            self.canvas = Canvas(opacity=self.opacity)
        self.bind(pos=self.update_canvas, size=self.update_canvas)
        self.bind(angle=self.update_canvas, magnitude=self.update_canvas)
        Clock.schedule_once(lambda dt: self.update_canvas())

    def update_canvas(self, *args):
        if self.width == 0 or self.height == 0:
            return

        self.radius = self.width // 2
        self.knob_radius = self.width // 6

        center_x = self.x + self.width / 2
        center_y = self.y + self.height / 2

        if not self.user_has_touched:
            self.knob_x = center_x
            self.knob_y = center_y

        self.canvas.clear()
        with self.canvas:
            Color(0.3, 0.3, 0.3, 0.5)
            Ellipse(
                pos=(center_x - self.radius, center_y - self.radius),
                size=(self.radius * 2, self.radius * 2),
            )
            Color(0.5, 0.5, 0.5, 0.8)
            Line(circle=(center_x, center_y, self.radius), width=2)

            Color(0.8, 0.6, 0.4, 0.8)
            Ellipse(
                pos=(self.knob_x - self.knob_radius, self.knob_y - self.knob_radius),
                size=(self.knob_radius * 2, self.knob_radius * 2),
            )
            Color(0.9, 0.5, 0.3, 1.0)
            Line(circle=(self.knob_x, self.knob_y, self.knob_radius), width=2)

    def on_touch_down(self, touch):
        dx = touch.x - self.center_x
        dy = touch.y - self.center_y
        distance = math.sqrt(dx**2 + dy**2)
        if distance <= self.radius:
            touch.grab(self)
            self.user_has_touched = True
            self.process_touch(touch)
            return True
        return super().on_touch_down(touch)

    def on_touch_move(self, touch):
        if touch.grab_current is self:
            self.process_touch(touch)
            return True
        return super().on_touch_move(touch)

    def on_touch_up(self, touch):
        if touch.grab_current is self:
            touch.ungrab(self)
            self.user_has_touched = False
            self.knob_x = self.center_x
            self.knob_y = self.center_y
            self.angle = 0
            self.magnitude = 0
            self.update_canvas()
            return True
        return super().on_touch_up(touch)

    def process_touch(self, touch):
        dx = touch.x - self.center_x
        dy = touch.y - self.center_y
        distance = math.sqrt(dx**2 + dy**2)
        max_distance = self.radius - self.knob_radius - 5
        if distance > max_distance:
            scale = max_distance / distance
            dx = dx * scale
            dy = dy * scale
            distance = max_distance
        self.knob_x = self.center_x + dx
        self.knob_y = self.center_y + dy
        self.angle = math.degrees(math.atan2(dy, dx)) - 90
        self.magnitude = min(distance / max_distance, 1.0)
        self.update_canvas()
