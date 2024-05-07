from __future__ import annotations
from pathlib import Path
from typing import Callable
import numpy as np
from ambf_client import Client
import time
from dataclasses import dataclass, field
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.transform import Rotation
from ambf_base_object import BaseObject
import click
import sys


def plot_3d_traj(traj_gen: CircularTrajectory):
    points = traj_gen.to_np_array().T
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(points[0], points[1], points[2], marker="x")
    ax.scatter(*points.T[0], color="red")
    plt.show()


@dataclass
class CircularTrajectory:
    radius: float
    center: np.ndarray
    num_points: int
    a_vec: np.ndarray = field(default_factory=lambda: np.array([1, 0, 0]))
    b_vec: np.ndarray = field(default_factory=lambda: np.array([0, 1, 0]))

    def __post_init__(self):
        self.create_trajectory()

    def create_trajectory(self) -> None:
        self._inner_array = []

        for theta in np.linspace(0, 2 * np.pi, self.num_points):
            self._inner_array.append(self.get_point(theta))

    def get_point(self, theta: float) -> np.ndarray:
        new_point = self.radius * np.array([np.cos(theta), np.sin(theta), 0])
        new_point = self.center + new_point

        return new_point

    def __len__(self) -> int:
        return len(self._inner_array)

    def __getitem__(self, index: int) -> np.ndarray:
        return self._inner_array[index]

    def to_np_array(self) -> np.ndarray:
        return np.array(self._inner_array)


def convert_point_msg_to_array(point_msg: Point) -> np.ndarray:
    return np.array([point_msg.x, point_msg.y, point_msg.z])


def rot_2_matrix(rot: np.ndarray) -> np.ndarray:
    r = Rotation.from_matrix(rot)

    # quaternion [x,y,z,w]
    quaternion = r.as_quat()

    return quaternion


def calculate_camera_orientation(gripper_handle, new_origin):
    """
    Camera frame is looking to the -z direction. This axis should always be looking at the object.
    """
    gripper_pos = convert_point_msg_to_array(gripper_handle.get_pos())
    y = np.array([0, 0, 1])
    z = -np.array(gripper_pos - new_origin)
    z = z / np.linalg.norm(z)
    x = np.cross(y, z)

    rotation = np.array([x, y, z]).T
    return rotation


def play_trajectory(
    cam_frame_handle: BaseObject,
    tool_yaw_link_handle: BaseObject,
    trajectory_gen: CircularTrajectory,
    after_motion_cb: Callable = None,
):
    for i in range(len(trajectory_gen)):
        new_origin = trajectory_gen[i]
        new_rot = calculate_camera_orientation(tool_yaw_link_handle, new_origin)
        new_rot = rot_2_matrix(new_rot)

        cam_frame_handle.set_pos(new_origin[0], new_origin[1], new_origin[2])
        cam_frame_handle.set_rot(new_rot)

        time.sleep(0.4)

        if after_motion_cb is not None:
            after_motion_cb()


def init_data_recorder():
    """In progress"""
    # from ambf6dpose.DataCollection.SimulatorDataProcessor import SimulatorDataProcessor
    # from ambf6dpose.DataCollection.BOPSaver.BopSaver import BopSampleSaver
    # from ambf6dpose.DataCollection.RosClients import SyncRosInterface

    # ros_client = SyncRosInterface()
    # samples_generator = SimulatorDataProcessor(ros_client)

    # def wait_for_data(client: SyncRosInterface):
    #     try:
    #         client.wait_for_data(28)
    #     except TimeoutError:
    #         print(
    #             "ERROR: Timeout exception triggered. ROS message filter did not receive any data.",
    #             file=sys.stderr,
    #         )
    #         sys.exit(1)

    # return samples_generator
    return None


def gen_data_recorder_cb(path: Path):

    assert path is not None, "recording path cannot be None"

    # samples_generator = init_data_recorder()

    print(f"recording data to {path}")

    def callback():
        print("Recording data...")

    return callback


@click.command("cli", context_settings={"show_default": True})
@click.option("--radius", default=0.08, help="Radius of the circular trajectory")
@click.option("--num_points", default=30, help="Number of points in the trajectory")
@click.option("--plot", is_flag=True, help="Plot the trajectory", default=False)
@click.option("--record", is_flag=True, help="Record the trajectory", default=False)
@click.option(
    "--path",
    default=None,
    type=click.Path(path_type=Path),
    help="Path to record the data",
)
def main(radius: float, num_points: int, plot: bool, record: bool, path: Path):
    c = Client("CircularTrajectory")
    c.connect()
    time.sleep(0.4)

    cam_frame_handle: BaseObject = c.get_obj_handle("CameraFrame")
    tool_yaw_link_handle: BaseObject = c.get_obj_handle("tool_yaw_link_001")

    center = convert_point_msg_to_array(tool_yaw_link_handle.get_pos())
    trajectory_gen = CircularTrajectory(
        radius=radius, center=center, num_points=num_points
    )
    data_rec_cb = None

    if plot:
        plot_3d_traj(trajectory_gen)
    if record:
        data_rec_cb = gen_data_recorder_cb(path)

    play_trajectory(
        cam_frame_handle,
        tool_yaw_link_handle,
        trajectory_gen,
        after_motion_cb=data_rec_cb,
    )


if __name__ == "__main__":
    main()
