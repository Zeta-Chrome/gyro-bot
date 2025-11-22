"""
Custom triangle button widget
"""
from kivy.clock import Clock
from kivy.graphics import Canvas, Color, Line, Triangle
from kivy.properties import StringProperty
from kivy.uix.widget import Widget


class TriangleButton(Widget):
    """Custom triangle button widget"""

    direction = StringProperty("up")  # 'up' or 'down'

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.pressed = False
        if self.canvas is None:
            self.canvas = Canvas()
        self.bind(pos=self.update_canvas, size=self.update_canvas)
        self.bind(direction=self.update_canvas)
        Clock.schedule_once(lambda dt: self.update_canvas())

    def update_canvas(self, *args):
        if self.width == 0 or self.height == 0:
            return

        self.canvas.clear()
        with self.canvas:
            # Background color changes when pressed
            if self.pressed:
                Color(0.8, 0.6, 0.4, 1.0)
            else:
                Color(0.7, 0.5, 0.3, 1.0)

            # Draw triangle
            cx = self.x + self.width / 2
            cy = self.y + self.height / 2
            w = self.width * 0.6
            h = self.height * 0.6

            if self.direction == "up":
                points = [
                    cx,
                    cy + h / 2,  # Top point
                    cx - w / 2,
                    cy - h / 2,  # Bottom left
                    cx + w / 2,
                    cy - h / 2,  # Bottom right
                ]
            else:  # down
                points = [
                    cx,
                    cy - h / 2,  # Bottom point
                    cx - w / 2,
                    cy + h / 2,  # Top left
                    cx + w / 2,
                    cy + h / 2,  # Top right
                ]

            Triangle(points=points)

            # Border
            Color(0.8, 0.8, 0.8, 1.0)
            Line(points=points + [points[0], points[1]], width=2)

    def on_touch_down(self, touch):
        if self.collide_point(*touch.pos):
            self.pressed = True
            self.update_canvas()
            touch.grab(self)
            return True
        return super().on_touch_down(touch)

    def on_touch_up(self, touch):
        if touch.grab_current is self:
            self.pressed = False
            self.update_canvas()
            touch.ungrab(self)
            return True
        return super().on_touch_up(touch)
