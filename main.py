import time
from math import sqrt

import ustruct
from machine import time_pulse_us, Pin, I2C, PWM
from micropython import const

DEVICE_ADDRESS = const(0b1101011)


def map_from_to(val, in_low, in_high, out_low, out_high):
    y = (val - in_low) / (in_high - in_low) * (out_high - out_low) + out_low
    return y


def normalize_vec(x, y, z):
    length = sqrt(x ** 2 + y ** 2 + z ** 2)
    return x / length, y / length, z / length


class RCReceiver:
    def __init__(self, pins: list[Pin | int], value_range: tuple[int, int] | None = None):
        if len(pins) < 1 or len(pins) > 6:
            raise Exception("wrong number of pins", pins)
        pins: list[Pin] = [Pin(i, Pin.IN) for i in pins if not isinstance(i, Pin)]
        self.pins = pins
        self.value_range = value_range

    def get_raw(self) -> list[int]:
        return [time_pulse_us(i, 3_000_000) for i in self.pins]

    def get_mapped(self) -> list[float]:
        if self.value_range is not None:
            low, high = self.value_range
            return [map_from_to(i, low, high, 0, 1) for i in self.get_raw()]
        else:
            return self.get_raw()


class Gyro:
    def __init__(self, slice_id: int, signal: Pin | int, clock: Pin | int, freq: int = 400_000):
        if not isinstance(signal, Pin):
            signal = Pin(signal)
        if not isinstance(clock, Pin):
            clock = Pin(clock)
        self.avg_x = 0
        self.avg_y = 0
        self.avg_z = 0
        self.i2c = I2C(slice_id, scl=clock, sda=signal, freq=freq)
        self.i2c.writeto_mem(DEVICE_ADDRESS, 0x20, 0b10001111.to_bytes())

    def read(self):
        x_low = self.i2c.readfrom_mem(DEVICE_ADDRESS, 0x28, 2)
        # x_high = self.i2c.readfrom_mem(DEVICE_ADDRESS, 0x29, 1)[0]
        y_low = self.i2c.readfrom_mem(DEVICE_ADDRESS, 0x2a, 2)
        # y_high = self.i2c.readfrom_mem(DEVICE_ADDRESS, 0x2b, 1)[0]
        z_low = self.i2c.readfrom_mem(DEVICE_ADDRESS, 0x2c, 2)
        # z_high = self.i2c.readfrom_mem(DEVICE_ADDRESS, 0x2d, 1)[0]
        x: int = ustruct.unpack("<h", x_low)[0]
        y: int = ustruct.unpack("<h", y_low)[0]
        z: int = ustruct.unpack("<h", z_low)[0]

        # x = int.from_bytes(x_low, "little", signed=True)
        # y = int.from_bytes(y_low, "little", signed=True)
        # z = int.from_bytes(z_low, "little", signed=True)
        return x, y, z

    def calibrate(self, count=10_000):
        x_sum = 0
        y_sum = 0
        z_sum = 0
        for i in range(count):
            x, y, z = self.read()
            x_sum += x
            y_sum += y
            z_sum += z
        self.avg_x = x_sum // count
        self.avg_y = y_sum // count
        self.avg_z = z_sum // count

    def read_calibrated(self) -> tuple[int, int, int]:
        x, y, z = self.read()
        return x - self.avg_x, y - self.avg_y, z - self.avg_z

    def read_calibrated_mapped(self) -> tuple[float, float, float]:
        x, y, z = self.read_calibrated()
        return (
            map_from_to(x, -32768, 32767, -1, 1),
            map_from_to(y, -32768, 32767, -1, 1),
            map_from_to(z, -32768, 32767, -1, 1),
        )


class Drives:
    def __init__(self, front_left: Pin | int, front_right: Pin | int, rear_right: Pin | int, rear_left: Pin | int,
                 freq: int = 50):
        self.front_left = PWM(front_left, freq=freq)
        self.front_right = PWM(front_right, freq=freq)
        self.rear_right = PWM(rear_right, freq=freq)
        self.rear_left = PWM(rear_left, freq=freq)

    def set(self, yaw: float, pitch: float, roll: float, throttle: float):
        front_left_power = throttle + pitch + roll - yaw
        front_right_power = throttle + pitch - roll + yaw
        rear_right_power = throttle - pitch - roll - yaw
        rear_left_power = throttle - pitch + roll + yaw
        self.front_left.duty_u16(int(front_left_power * 65535))
        self.front_right.duty_u16(int(front_right_power * 65535))
        self.rear_right.duty_u16(int(rear_right_power * 65535))
        self.rear_left.duty_u16(int(rear_left_power * 65535))


class PID:
    def __init__(self, kp: float, ki: float, kd: float, cycle_time_seconds: float, i_limit: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.cycle_time_seconds = cycle_time_seconds
        self.i_limit = i_limit
        self.previous_integral: float = 0
        self.previous_error: float = 0

    def update(self, goal: float, current: float):
        error = goal - current
        proportional = error * self.kp
        integral = self.previous_integral + (error * self.ki * self.cycle_time_seconds)
        integral = max(min(integral, self.i_limit), -self.i_limit)
        derivative = (error - self.previous_error) * self.kd / self.cycle_time_seconds
        self.previous_integral = integral
        self.previous_error = error
        return proportional + integral + derivative

    def reset(self):
        self.previous_integral = 0
        self.previous_error = 0


current_yaw: float = 0
current_pitch: float = 0
current_roll: float = 0
gyro = Gyro(1, 0, 1)
gyro.calibrate()
CYCLE_FREQ = const(400)
CYCLE_TIME = const(1 / CYCLE_FREQ)
drives = Drives(0, 1, 2, 3)
pid_yaw = PID(1, 0, 0, CYCLE_TIME, 1)
pid_pitch = PID(1, 0, 0, CYCLE_TIME, 1)
pid_roll = PID(1, 0, 0, CYCLE_TIME, 1)
receiver = RCReceiver([4, 5, 6, 7], (1_000, 2_000))
while True:
    loop_start = time.ticks_us()
    yaw_change, pitch_change, roll_change = gyro.read_calibrated_mapped()
    current_yaw += yaw_change
    current_pitch += pitch_change
    current_roll += roll_change
    [desired_yaw, desired_pitch, desired_roll, desired_throttle] = receiver.get_mapped()
    yaw_power = pid_yaw.update(desired_yaw, current_yaw)
    pitch_power = pid_pitch.update(desired_pitch, current_pitch)
    roll_power = pid_roll.update(desired_roll, current_roll)
    drives.set(yaw_power, pitch_power, roll_power, desired_throttle)
    loop_end = time.ticks_us()
    loop_duration = time.ticks_diff(loop_end, loop_start)
    if loop_duration < CYCLE_TIME * 1_000_000:
        time.sleep_us(int(CYCLE_TIME * 1_000_000 - loop_duration))
