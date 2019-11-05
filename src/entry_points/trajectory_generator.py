import os
import sys
import pickle
import pathfinder as pf
import wpilib
from . import robotpy_entry_point
import math


WHEELBASE_WIDTH = 1.83  # In feet
TRAJECTORY_DIRECTORY = 'trajectories'
PICKLE_FILE = os.path.join(os.path.dirname(sys.modules['__main__'].__file__), TRAJECTORY_DIRECTORY, 'trajectories.pickle')
MAX_GENERATION_VELOCITY = 3.4
MAX_GENERATION_ACCELERATION = 8.5
MAX_GENERATION_JERK = 15

trajectories = {
    "charge": [
        pf.Waypoint(0, 0, 0),
        pf.Waypoint(3, 0, 0)
    ],
    "diagonal_higher": [
        pf.Waypoint(0, 0, 0),  # Waypoints are relative to first, so start at 0, 0, 0
        pf.Waypoint(15, 8, 0)
    ],
    "cargo_ship": [
        pf.Waypoint(0, 0, 0),
        pf.Waypoint(7.33, 0, 0)
    ],
    "left-side": [
        pf.Waypoint(0, 0, 0),
        pf.Waypoint(6, 6, math.pi / 4)
    ],
    "turn": [
        pf.Waypoint(0, 0, 0),  # Waypoints are relative to first, so start at 0, 0, 0
        pf.Waypoint(0.5, 0, math.pi / 4)
    ]
}


def load_trajectories():
    """
    Either generate and write trajectories if in a sim or read them if on the robot.
    """
    try:
        with open(PICKLE_FILE, 'rb') as f:
            generated_trajectories = pickle.load(f)
    except FileNotFoundError:
        generated_trajectories = {}

    return generated_trajectories


def _write_trajectories(trajectories):
    """
    Write trajectories dictionary to a file.
    :param trajectories: The trajectory dict to write.
    """
    with open(PICKLE_FILE, 'wb') as f:
        pickle.dump(trajectories, f)

    # if wpilib.RobotBase.isSimulation():
    #     from pyfrc.sim import get_user_renderer
    #
    #     renderer = get_user_renderer()
    #     if renderer:
    #         renderer.draw_pathfinder_trajectory(modifier.getLeftTrajectory(), '#0000ff', offset=(-0.9, 0))
    #         renderer.draw_pathfinder_trajectory(modifier.source, '#00ff00', show_dt=True)
    #         renderer.draw_pathfinder_trajectory(modifier.getRightTrajectory(), '#0000ff', offset=(0.9, 0))


@robotpy_entry_point(name='TrajectoryGenerator')
def generate_trajectories(options, robot_class):
    """
    Generate trajectory from waypoints.

    :param options: Options for the entry point being called
    :param robot_class: The class of the robot being run
    """
    print('Generating Trajectories...')
    generated_trajectories = {}

    for trajectory_name, trajectory in trajectories.items():
        print(f'Trajectory: {trajectory_name} ...', end='')
        generated_trajectory = pf.generate(
            trajectory,
            pf.FIT_HERMITE_CUBIC,
            pf.SAMPLES_HIGH,
            dt=0.02,  # 20ms
            max_velocity=MAX_GENERATION_VELOCITY,  # These are in ft/sec and
            max_acceleration=MAX_GENERATION_ACCELERATION,  # set the units for distance to ft.
            max_jerk=MAX_GENERATION_JERK
        )[1]  # The 0th element is just info

        modifier = pf.modifiers.TankModifier(generated_trajectory).modify(WHEELBASE_WIDTH)

        generated_trajectories[trajectory_name] = (
            modifier.getLeftTrajectory(),
            modifier.getRightTrajectory()
        )
        print('Done')

    _write_trajectories(generated_trajectories)
    print('Finished.')