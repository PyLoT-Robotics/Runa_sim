"""Pan/tilt ticks and joint radians; no ROS dependency."""
import math

def tick_to_angle(servo_id, value):
    if servo_id == 1:
        return (1200 - max(0, min(1200, value))) * 2 * math.pi / 4096
    if servo_id == 2:
        return max(-2.9, min(2.9, (value - 1900) * 2 * math.pi / 4096))
    raise ValueError('pan/tilt ID must be 1 or 2')


def angle_to_tick(servo_id, value):
    return round(1200 - value * 4096 / (2 * math.pi)) if servo_id == 1 else round(1900 + value * 4096 / (2 * math.pi))
