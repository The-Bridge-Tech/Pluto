"""
Servo PWM Controller Class
Author: Matthew Lauriault
Created: 10/2/24
"""


# ROS MODULES
from rclpy.node import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger
from std_msgs.msg import UInt32


class PWM:

        """Struct controller class for servo pwm"""

        def __init__(self, neutral: int, min: int, max: int, publisher: Publisher, logger: RcutilsLogger):
                self._value = neutral
                self.neutral = neutral
                self.min = min
                self.max = max
                self.publisher = publisher
                self.logger = logger

        @property
        def value(self) -> int:
                return self._value
        @value.setter
        def value(self, value: int) -> None:
                # if above max -> set to max
                if value > self.max:
                        self.logger.warn("Attempted to set servo pwm above max.")
                        self._value = self.max
                # if below min -> set to min
                elif value < self.min:
                        self.logger.warn("Attempted to set servo pwm below min.")
                        self._value = self.min
                # within min and max -> set
                else:
                        self._value = round(value)
                # publish new value
                self.publisher.publish(UInt32(data=self._value))
        
        @property
        def percentage(self) -> int:
                if self.value == self.neutral:
                        return 0
                elif self.value < self.neutral:
                        return ((self.value - self.neutral) / (self.neutral - self.min)) * 100
                elif self.value > self.neutral:
                        return ((self.value - self.neutral) / (self.max - self.neutral)) * 100
        @percentage.setter
        def percentage(self, value: int|float):
                if value == 0:
                        self.set_neutral()
                elif value < 0:
                        if value < -100:
                                self.logger.warn("Attempted to set servo pwm below -100%.")
                                value = -100
                        self.value = self.neutral + ((self.neutral - self.min) * (value / 100))
                elif value > 0:
                        if value > 100:
                                self.logger.warn("Attempted to set servo pwm above 100%.")
                                value = 100
                        self.value = self.neutral + ((self.max - self.neutral) * (value / 100))

        def set_neutral(self) -> None:
                self.value = self.neutral