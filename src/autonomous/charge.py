from magicbot.state_machine import state, timed_state, AutonomousStateMachine
from src.components import drive, trajectory_follower, lift


class Charge(AutonomousStateMachine):
    MODE_NAME = 'Charge'

    drive = drive.Drive
    lift: lift.Lift
    follower: trajectory_follower.TrajectoryFollower

    @state(first=True)
    def charge(self, initial_call):
        if initial_call:
            self.follower.follow_trajectory('left-side')

        if not self.follower.is_following('left-side'):
            self.done()  # If using mutliple states use self.next_state(name)
