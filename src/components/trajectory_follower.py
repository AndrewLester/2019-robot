import pathfinder as pf
import navx
from typing import Tuple, List
import wpilib
from . import drive
from magicbot.magic_tunable import tunable
from ctre.wpi_talonsrx import WPI_TalonSRX


class TrajectoryFollower:
    """
    Move along generated paths for autonomous
    """
    WHEEL_DIAMETER = 0.5 # Units: Feet
    KV = tunable(0.831)
    KA = tunable(0.033)
    ANGLE_KP = tunable(1)
    ANGLE_KD = tunable(0.25)

    drive: drive.Drive
    navx: navx.AHRS
    l_encoder: wpilib.Encoder
    r_encoder: wpilib.Encoder
    generated_trajectories: dict

    def on_enable(self):
        """
        Setup encoder follower objects.
        """
        self._current_trajectory = None
        self.last_difference = 0

        self.left_follower = pf.followers.EncoderFollower(None)
        self.right_follower = pf.followers.EncoderFollower(None)

        self.left_follower.configurePIDVA(1, 0, 0.1, self.KV, self.KA)
        self.right_follower.configurePIDVA(1, 0, 0.1, self.KV, self.KA)

        self._configure_encoders()

    def follow_trajectory(self, trajectory_name: str):
        """
        Follow a specified trajectory.
        :param trajectory_name: The name of the trajectory to follow.
        """
        if trajectory_name not in self.generated_trajectories.keys():
            return

        print('Following Trajectory:', trajectory_name)
        self._current_trajectory = trajectory_name

        self._configure_encoders()
        self.left_follower.setTrajectory(self.generated_trajectories[trajectory_name][0])
        self.right_follower.setTrajectory(self.generated_trajectories[trajectory_name][1])

    def _configure_encoders(self):
        """
        Configure the encoders for following a trajectory.
        """
        self.left_follower.configureEncoder(self.l_encoder.get(), 1024, self.WHEEL_DIAMETER)
        self.right_follower.configureEncoder(self.r_encoder.get(), 1024, self.WHEEL_DIAMETER)

    def is_following(self, trajectory_name):
        """
        Check whether a trajectory is being followed.
        :param trajectory_name: The name of the trajectory to check.
        """
        return self._current_trajectory is not None and self._current_trajectory == trajectory_name

    def execute(self):
        """
        Calculate the movement values and move the robot.
        """
        if (self.left_follower.trajectory is None or self.right_follower.trajectory is None) or \
           (self.left_follower.isFinished() and self.right_follower.isFinished()):
            self._current_trajectory = None
            return

        left = self.left_follower.calculate(self.l_encoder.get())
        right = self.right_follower.calculate(self.r_encoder.get())

        gyro_heading = -self.navx.getAngle()  # Assuming the gyro is giving a value in degrees
        desired_heading = pf.r2d(self.left_follower.getHeading())  # Should also be in degrees

        angle_difference = pf.boundHalfDegrees(desired_heading - gyro_heading)
        print(f'{desired_heading} - {gyro_heading} = {angle_difference}')
        turn = (self.ANGLE_KP * angle_difference) + (self.ANGLE_KD * (angle_difference - self.last_difference))

        self.last_difference = angle_difference
        print(f'Turn: {turn}')
        left -= turn
        right += turn

        print('Drive:', left, right)
        # print('Encoders:', self.l_encoder.get(), self.r_encoder.get())

        self.drive.move_tank(left, right)
