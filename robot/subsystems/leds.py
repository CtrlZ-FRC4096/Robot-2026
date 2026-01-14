from typing import TYPE_CHECKING

# from commands2 import SubsystemBase
from wpilibextra.coroutine.subsystem import Subsystem

if TYPE_CHECKING:
    from robot import Robot

import wpilib

# Constants
PWM_PORT_STRIP_1 = 0
PWM_PORT_STRIP_2 = 1
NUM_LEDS = 18


class LEDs(Subsystem):
    # LED pattern modes to cycle through
    MODE_ODOMETRY = "odometry"  # blue, solid (there was "purple bad here", but I am unsure what odometry mode means)
    MODE_LOST_ODOMETRY = "lost odometry" # orange, solid
    # if both odometry and cams are lost, the color is red, blinking
    MODE_LOCKED_ON = "locked on" # white, solid
    MODE_INTAKING = "intaking" # yellow, solid

    # RGB Colors
    COLOR_WHITE = (255, 255, 255)
    COLOR_RED = (255, 0, 0)
    COLOR_ORANGE = (246, 61, 25)
    COLOR_YELLOW = (255, 255, 0)
    COLOR_GREEN = (0, 255, 0)
    COLOR_CYAN = (0, 255, 255)
    COLOR_BLUE = (0, 0, 255)
    COLOR_MAGENTA = (255, 0, 255)

    def __init__(self, robot: "Robot"):
        super().__init__()
        self.robot = robot

        self.mode = self.MODE_ODOMETRY

        self.led_strip_1 = wpilib.AddressableLED(port=0)  # check this
        # self.led_strip_2 = wpilib.AddressableLED(port=PWM_PORT_STRIP_2)

        self.led_strip_1.setLength(NUM_LEDS)  # check this
        # self.led_strip_2.setLength(NUM_LEDS)

        self.data = []
        for i in range(NUM_LEDS):
            self.data.append(wpilib.AddressableLED.LEDData(0, 0, 0))

        self.led_strip_1.setData(self.data)
        self.led_strip_1.start()

        # self.led_strip_2.setData(self.data)
        # self.led_strip_2.start()

        # States & timers related to patterns below
        self.timer1 = wpilib.Timer()
        self.timer1.start()
        self.flash_on = True
        self.pulse_color_idx = 0
        self.pulse_color = (0, 0, 0)
        self.scroll_color_idx = 0
        self.scroll_color = (0, 0, 0)
        self.scroll_pos = 0

    def stop(self):
        pass
        # self.robot.nt_robot.putString('led_mode', self.MODE_OFF)

    def get_mode(self) -> str:
        return self.mode

    def set_mode(self, mode):
        self.mode = mode
        self.robot.nt_robot.putString("led_mode", mode)

    def fill(self, color):
        for led in self.data:
            led.setRGB(*color)

        self.led_strip_1.setData(self.data)
        # self.led_strip_2.setData(self.data)

    def clear(self):
        self.fill((0, 0, 0))
        self.led_strip_1.setData(self.data)
        # self.led_strip_2.setData(self.data)

    def pattern_scroll(self, colors, steps=8):
        multiplier = 0.7

        if self.scroll_color == (0, 0, 0):
            self.scroll_color = list(colors[self.scroll_color_idx])

        # Start by setting cur_color to scroll_color
        cur_color = tuple([int(c * multiplier) for c in self.scroll_color])
        i = 0

        for n in range(steps):
            # print('cur_color =', cur_color)
            i = self.scroll_pos - n

            if i < 0 or i >= NUM_LEDS:
                continue

            self.data[i].setRGB(*cur_color)

            # Fade cur_color a little
            cur_color = tuple([int(c * multiplier) for c in cur_color])

        if i > 0:
            self.data[i - 1].setRGB(0, 0, 0)

        self.led_strip_1.setData(self.data)
        # self.led_strip_2.setData(self.data)

        self.scroll_pos += 1

        if self.scroll_pos - steps >= NUM_LEDS:
            self.scroll_color = (0, 0, 0)
            self.scroll_pos = 0
            self.scroll_color_idx += 1

            if self.scroll_color_idx == len(colors):
                self.scroll_color_idx = 0

    def pattern_pulse(self, colors):
        # Arbitrary amount to fade color each cycle
        multiplier = 0.7

        if self.pulse_color == (0, 0, 0):
            self.pulse_color = list(colors[self.pulse_color_idx])

        self.pulse_color = tuple([int(c * multiplier) for c in self.pulse_color])
        self.fill(self.pulse_color)

        if sum(self.pulse_color) / 3.0 < 20:
            # Move to next color
            self.pulse_color == (0, 0, 0)
            self.pulse_color_idx += 1

            if self.pulse_color_idx == len(colors):
                self.pulse_color_idx = 0

    def pattern_flash(self, color):
        for i in range(10):
            if self.timer1.hasElapsed(0.05):
                if self.flash_on:
                    self.clear()
                else:
                    self.fill(color)

                self.flash_on = not self.flash_on
                self.timer1.reset()

    def periodicX(self):
        # Set colors for current LED mode

        if self.mode == self.MODE_ODOMETRY:
            self.fill(self.COLOR_BLUE)

        elif self.mode == self.MODE_LOST_ODOMETRY:
            self.fill(self.COLOR_ORANGE)

        elif self.mode == self.MODE_LOCKED_ON:
            self.fill(self.COLOR_WHITE)

        elif self.mode == self.MODE_INTAKING:
            self.fill(self.COLOR_YELLOW)

        else:
            pass

    def log(self):
        wpilib.SmartDashboard.putString("LEDs Mode", self.get_mode())