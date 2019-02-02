#!/usr/bin/env python3

import magicbot
import wpilib
import wpilib.drive
import math


class TestRobot(magicbot.MagicRobot):
    ENCODER_PULSE_PER_REV = 1024
    WHEEL_DIAMETER = 0.5

    def createObjects(self):
        """
        Initialize testbench components.
        """
        self.joystick = wpilib.Joystick(0)
        self.lift_motors = [wpilib.Victor(i) for i in range(4, 8)]

        encoder_constant = (
            (1 / self.ENCODER_PULSE_PER_REV) * self.WHEEL_DIAMETER * math.pi
        )

        self.l_encoder = wpilib.Encoder(2, 3)
        self.l_encoder.setDistancePerPulse(encoder_constant)
        self.r_encoder = wpilib.Encoder(0, 1)
        self.r_encoder.setDistancePerPulse(encoder_constant)

    def teleopPeriodic(self):
        """
        Spin all motors at full speed.
        """
        # for motor in self.lift_motors:
        #     motor.set(self.joystick.getY())

        print(self.r_encoder.get())


if __name__ == '__main__':
    wpilib.run(TestRobot)
