from typing import List
import numpy as np
import subprocess
from pathlib import Path
import shutil
from tensorboardX import SummaryWriter


ROOT_FOLDER = Path('/root/social_gym')
DATA_FOLDER = ROOT_FOLDER / 'data'  # This is how it is set up in the docker container, not on your local machine


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


def get_tboard_writer(log_name: str):
    logdir = Path(f'{DATA_FOLDER}/{log_name}')

    subprocess.Popen(["sudo", "chmod", "-R", "a+rwX",  f"{logdir}"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    if logdir.exists():
        shutil.rmtree(str(logdir))

    logdir.mkdir(exist_ok=True, parents=True)

    print(f"Making tensorboard summary writer at {logdir}")

    return SummaryWriter(logdir=str(logdir)), logdir
