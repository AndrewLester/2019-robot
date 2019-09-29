import math
from pyfrc.physics import drivetrains


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Mecanum Drive joystick control
    """

    def __init__(self, physics_controller):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                       to communicate simulation effects to
        """
        self.physics_controller = physics_controller
        self.physics_controller.add_device_gyro_channel('navxmxp_spi_4_angle')
        self.encoder_ticks = 1440 / (0.5 * math.pi)
        self.drivetrain = drivetrains.MecanumDrivetrain(y_wheelbase=1.83, x_wheelbase=0.75, speed=3.4)
        self.left_encoder = 0
        self.right_encoder = 0

    def update_sim(self, hal_data, now, tm_diff):
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        # Invert right side because it is inverted in the mecanum drive method
        lf_motor = hal_data['CAN'][10]['value']
        lr_motor = hal_data['CAN'][15]['value']
        rf_motor = -hal_data['CAN'][20]['value']
        rr_motor = -hal_data['CAN'][25]['value']

        vx, vy, vw = self.drivetrain.get_vector(lr_motor, rr_motor, lf_motor, rf_motor)

        self.physics_controller.vector_drive(vx, vy, vw, tm_diff)

        self.left_encoder += self.drivetrain.lr_speed * tm_diff
        self.right_encoder += self.drivetrain.rr_speed * tm_diff

        hal_data['encoder'][0]['count'] = int(self.right_encoder * self.encoder_ticks)
        hal_data['encoder'][1]['count'] = int(self.left_encoder * self.encoder_ticks)
