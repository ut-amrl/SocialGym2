import subprocess


def step_1_setup_git_branch():
    out = subprocess.Popen(['git', 'checkout', 'feature/20221218_multi_agent_finishing_and_cleanup'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out.wait()

    out = subprocess.Popen(['git submodule update --init --recursive'.split()], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out.wait()


if __name__ == "__main__":
    step_1_setup_git_branch()