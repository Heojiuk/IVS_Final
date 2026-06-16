"""TrackCanvas: tkinter Canvas widget for track + two-vehicle animation."""
import math

# Track geometry (world meters, center=origin)
_TRACK_OVALS = [
    (1.45, 1.225, '#22aa22', 3),   # outer green
    (1.25, 1.025, '#ddcc00', 3),   # yellow (lane separator)
    (1.05, 0.825, '#22aa22', 3),   # inner green
]
_CANVAS_W, _CANVAS_H = 900, 580
_SCALE = 210   # px/m
_CX, _CY = _CANVAS_W // 2, _CANVAS_H // 2


def world_to_screen(wx, wy, cx=_CX, cy=_CY, scale=_SCALE):
    """World (m, y-up) → screen (px, y-down)."""
    return cx + wx * scale, cy - wy * scale


def apply_motion_model(x, y, heading, throttle, steer, dt, k_v, k_w):
    """Single-step simple proportional motion model.
    Returns (new_x, new_y, new_heading) in world meters/radians."""
    heading = heading + steer * k_w * dt
    x = x + throttle * k_v * math.cos(heading) * dt
    y = y + throttle * k_v * math.sin(heading) * dt
    return x, y, heading


def _triangle_points(cx, cy, heading, size=12):
    """Screen-space triangle polygon for a top-view vehicle."""
    pts = []
    for angle, dist in [(0, size), (2.356, size * 0.6), (-2.356, size * 0.6)]:
        a = heading + angle
        pts += [cx + dist * math.cos(a), cy - dist * math.sin(a)]
    return pts


try:
    import tkinter as tk

    class TrackCanvas(tk.Canvas):
        """Draws the oval track and animates Pi vehicle + Sim vehicle."""

        def __init__(self, parent, **kwargs):
            kwargs.setdefault('width', _CANVAS_W)
            kwargs.setdefault('height', _CANVAS_H)
            kwargs.setdefault('bg', '#f5f5f5')
            super().__init__(parent, **kwargs)

            self._pi_state = None
            self._sim_state = None

            self._pi_tag = 'pi_vehicle'
            self._sim_tag = 'sim_vehicle'
            self._trail_tag = 'trail'

            self._draw_track()
            self.bind('<Button-1>', self._on_click)

            self.k_v = 1.0
            self.k_w = 2.0

        def _draw_track(self):
            self.delete('track')
            for rx, ry, color, width in _TRACK_OVALS:
                sx1, sy1 = world_to_screen(-rx, ry)
                sx2, sy2 = world_to_screen(rx, -ry)
                self.create_oval(sx1, sy1, sx2, sy2, outline=color, width=width, tags='track')

        def _on_click(self, event):
            wx = (event.x - _CX) / _SCALE
            wy = (_CY - event.y) / _SCALE
            self._pi_state = [wx, wy, 0.0]
            self._sim_state = [wx, wy, 0.0]
            self._redraw_vehicles()

        def set_start_pos(self, wx, wy, heading=0.0):
            self._pi_state = [wx, wy, heading]
            self._sim_state = [wx, wy, heading]
            self._redraw_vehicles()

        def reset_trail(self):
            self.delete(self._trail_tag)

        def update_pi(self, throttle, steer, dt):
            if self._pi_state is None:
                return
            x, y, h = apply_motion_model(*self._pi_state, throttle, steer, dt, self.k_v, self.k_w)
            old_sx, old_sy = world_to_screen(*self._pi_state[:2])
            self._pi_state = [x, y, h]
            new_sx, new_sy = world_to_screen(x, y)
            self.create_line(old_sx, old_sy, new_sx, new_sy, fill='#4488ff', width=1, tags=self._trail_tag)
            self._redraw_vehicles()

        def update_sim(self, throttle, steer, dt):
            if self._sim_state is None:
                return
            x, y, h = apply_motion_model(*self._sim_state, throttle, steer, dt, self.k_v, self.k_w)
            old_sx, old_sy = world_to_screen(*self._sim_state[:2])
            self._sim_state = [x, y, h]
            new_sx, new_sy = world_to_screen(x, y)
            self.create_line(old_sx, old_sy, new_sx, new_sy, fill='#ff4444', width=1, tags=self._trail_tag)
            self._redraw_vehicles()

        def _redraw_vehicles(self):
            self.delete(self._pi_tag)
            self.delete(self._sim_tag)
            if self._pi_state:
                sx, sy = world_to_screen(*self._pi_state[:2])
                pts = _triangle_points(sx, sy, self._pi_state[2])
                self.create_polygon(pts, fill='#2255cc', outline='white', width=1, tags=self._pi_tag)
            if self._sim_state:
                sx, sy = world_to_screen(*self._sim_state[:2])
                pts = _triangle_points(sx, sy, self._sim_state[2])
                self.create_polygon(pts, fill='#cc2222', outline='white', width=1, tags=self._sim_tag)

except ImportError:
    # tkinter unavailable (headless/CI): TrackCanvas not usable, but pure functions still work
    class TrackCanvas:  # type: ignore[no-redef]
        def __init__(self, *args, **kwargs):
            raise RuntimeError("tkinter is not available in this environment")
