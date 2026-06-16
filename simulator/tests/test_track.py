import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from track_canvas import world_to_screen, apply_motion_model


def test_world_to_screen_center():
    sx, sy = world_to_screen(0.0, 0.0, cx=450, cy=320, scale=220)
    assert sx == 450
    assert sy == 320


def test_world_to_screen_right():
    sx, sy = world_to_screen(1.0, 0.0, cx=450, cy=320, scale=220)
    assert abs(sx - 670) < 1   # 450 + 220
    assert sy == 320


def test_world_to_screen_up_inverted():
    # y=1.0 world → y decreases in screen (screen y is inverted)
    sx, sy = world_to_screen(0.0, 1.0, cx=450, cy=320, scale=220)
    assert sx == 450
    assert abs(sy - 100) < 1   # 320 - 220


def test_motion_model_straight():
    x, y, h = apply_motion_model(0.0, 0.0, 0.0, throttle=1.0, steer=0.0, dt=1.0, k_v=1.0, k_w=1.0)
    assert abs(x - 1.0) < 1e-9
    assert abs(y) < 1e-9
    assert abs(h) < 1e-9


def test_motion_model_turn():
    x, y, h = apply_motion_model(0.0, 0.0, 0.0, throttle=0.0, steer=1.0, dt=1.0, k_v=1.0, k_w=math.pi/2)
    assert abs(h - math.pi/2) < 1e-9
    assert abs(x) < 1e-9   # no throttle → no position change


if __name__ == '__main__':
    test_world_to_screen_center()
    test_world_to_screen_right()
    test_world_to_screen_up_inverted()
    test_motion_model_straight()
    test_motion_model_turn()
    print('OK')
