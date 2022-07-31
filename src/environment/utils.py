from typing import List
import numpy as np


def poses_to_np_array(poses: List) -> np.array:
    """
    Given a list of poses, convert them into a numpy array.

    :param poses: List of poses
    :returns: np array where the values are the [ poses[0].x, poses[0].y, poses[0].theta, ..., poses[N].theta ]
    """

    # TODO - get the Pose2Df class imported so we can check for single instaces.
    if not isinstance(poses, list) and not isinstance(poses, tuple):
        poses = [poses]

    coord_list = []
    for pose in poses:
        coord_list.append(pose.x)
        coord_list.append(pose.y)
        coord_list.append(pose.theta)
    return np.array(coord_list)
