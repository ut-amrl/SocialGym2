import argparse
import json
import subprocess
from pathlib import Path
from typing import List, Tuple
from uuid import uuid4
import time
from tqdm import tqdm

ROOT_FOLDER =  Path(__file__).parent.absolute()
CONFIG_FOLDER = ROOT_FOLDER / 'configs'
TMP_FOLDER = ROOT_FOLDER / 'tmp'

def run(
        configs: List[Path],
        pid_file_tracker: Path = None,
        gpu: str = None,
        prefix_name: str = None
):

    container_names = []
    names_to_config = {}

    for config in tqdm(configs, total=len(configs), desc='Starting containers'):
        print("AAAAAAAAAAAAAAAA" + config)
        if gpu or prefix_name:
            c_file = CONFIG_FOLDER / config
            c = {}
            with c_file.open('r') as f:
                c = json.load(f)
            if gpu:
                c['device'] = gpu
            if prefix_name:
                c['run_name'] = f"{prefix_name}/{c['run_name'].split('/')[-1]}"

            with c_file.open('w') as f:
                json.dump(c, f)

        container_name = str(uuid4()).replace('-', '')
        subprocess.run([str(ROOT_FOLDER / 'run_config.sh'), container_name, f'/home/rosdev/social_gym/config_runner/configs/{config}'], stdout=subprocess.PIPE)

        container_names.append(container_name)
        names_to_config[container_name] = config
        time.sleep(10)

    print('Started the following containers')
    for config, name in zip(configs, container_names):
        print(f'{name}   ->  {config}')

    print()
    print("Logs can be seen via (remove -f to not follow the logs but just see what's been logged so far)")

    for config, name in zip(configs, container_names):
        print(f'docker logs -f {name}')

    print()
    print('Kill all these containers')
    print("; ".join([f"docker kill {x}" for x in container_names]))

    if pid_file_tracker:
        pid_file_tracker.parent.mkdir(exist_ok=True, parents=True)
        with pid_file_tracker.open('w') as f:
            json.dump(names_to_config, f)

    status = {k: False for k in container_names}

    # while any([not x for x in status.values()]):
    #     print('\n\n------ ------ ------ ------')
    #     print('Making sure processes are running')

    #     time.sleep(20)

    #     for k, v in status.items():
    #         if v:
    #             continue

    #         out = subprocess.check_output(['docker', 'logs', k], stderr=subprocess.PIPE)

    #         if 'started roslaunch server' in str(out):
    #             print(f'{k} started correctly!')
    #             status[k] = True
    #         else:
    #             print(f'{k} is being restarted!')
    #             subprocess.run(['docker', 'kill', k], stdout=subprocess.PIPE)
    #             subprocess.run(['docker', 'rm', k], stdout=subprocess.PIPE)

    #             subprocess.run([str(ROOT_FOLDER / 'run_config.sh'), k,
    #                             f'/home/rosdev/social_gym/config_runner/configs/{names_to_config[k]}'], stdout=subprocess.PIPE)

    #             print("Delaying for the container to load...")
    #             time.sleep(20)

    print('All containers are running!')


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument('--folder', '-f', type=str, help='Path to a directory that has a batch of json configs to run inside the {PROJECT_ROOT}/config_runner/configs folder.')
    parser.add_argument('--config', '-c', type=str, default=[], nargs='+', help='Individual config files to run inside the {PROJECT_ROOT}/config_runner/configs folder. Can be separated by spaces.')
    parser.add_argument('--pid_tracking_file', '-p', type=str, help='File that keeps track of the docker names and config files mapping.')

    parser.add_argument('--gpu', '-g', type=str)
    parser.add_argument('--name', '-n', type=str, help='File that keeps track of the docker names and config files mapping.')

    args = parser.parse_args()

    folder = args.folder
    configs = [x for x in args.config]
    pid_file = Path(args.pid_tracking_file) if args.pid_tracking_file else None

    gpu = args.gpu
    name = args.name

    if folder is not None:
        configs.extend([f'{folder}/{x.name}' for x in list((CONFIG_FOLDER / folder).glob("*.json"))])

    run(configs, pid_file_tracker=pid_file, gpu=gpu, prefix_name=name)



