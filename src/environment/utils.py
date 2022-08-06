from typing import List
import numpy as np
import subprocess
from pathlib import Path
import shutil
import re
import sys
from tensorboardX import SummaryWriter
import time


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
    time.sleep(1)

    logdir = Path(f'{DATA_FOLDER}/{log_name}')

    subprocess.Popen(["sudo", "chmod", "-R", "a+rwX",  f"{logdir}"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    if logdir.exists():
        shutil.rmtree(str(logdir))

    logdir.mkdir(exist_ok=True, parents=True)

    print(f"Making tensorboard summary writer at {logdir}")

    return SummaryWriter(logdir=str(logdir)), logdir


# TODO - This shouldn't be needed, the real fix is in changing the c++ code.
class LogFilter(object):
    """
    Taken from https://stackoverflow.com/questions/34904946/how-to-filter-stdout-in-python-logging
    """
    def __init__(self, stream, re_pattern):
        self.stream = stream
        self.pattern = re.compile(re_pattern) if isinstance(re_pattern, str) else re_pattern
        self.triggered = False

    def __getattr__(self, attr_name):
        return getattr(self.stream, attr_name)

    def write(self, data):
        if data == '\n' and self.triggered:
            self.triggered = False
        else:
            if self.pattern.search(data) is None:
                self.stream.write(data)
                self.stream.flush()
            else:
                # caught bad pattern
                self.triggered = True

    def flush(self):
        self.stream.flush()


def filter_stdout(filter: str):
    sys.stdout = LogFilter(sys.stdout, filter)  # filter out any line which contains "Read -1" in it
