import argparse
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
):

    container_names = []

    for config in tqdm(configs, total=len(configs), desc='Starting containers'):
        container_name = str(uuid4()).replace('-', '')
        subprocess.run([str(ROOT_FOLDER / 'run_config.sh'), container_name, f'/home/rosdev/social_gym/config_runner/configs/{config}'], stdout=subprocess.PIPE)

        container_names.append(container_name)

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

    status = {k: False for k in container_names}

    while any([not x for x in status.values()]):
        print('\n\n------ ------ ------ ------')
        print('Making sure processes are running')

        time.sleep(30)

        for k, v in status.items():
            if v:
                continue

            out = subprocess.check_output(['docker', 'logs', k], stderr=subprocess.PIPE)

            if 'started roslaunch server' in str(out):
                print(f'{k} started correctly!')
                status[k] = True
            else:
                print(f'{k} is being restarted!')
                subprocess.run(['docker', 'kill', k], stdout=subprocess.PIPE)
                subprocess.run(['docker', 'rm', k], stdout=subprocess.PIPE)

                subprocess.run([str(ROOT_FOLDER / 'run_config.sh'), k,
                                f'/home/rosdev/social_gym/config_runner/configs/{configs[container_names.index(k)]}'], stdout=subprocess.PIPE)


    print('All containers are running!')


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument('--folder', '-f', type=str, help='Path to a directory that has a batch of json configs to run inside the {PROJECT_ROOT}/config_runner/configs folder.')
    parser.add_argument('--config', '-c', type=str, default=[], nargs='+', help='Individual config files to run inside the {PROJECT_ROOT}/config_runner/configs folder. Can be separated by spaces.')

    args = parser.parse_args()

    folder = args.folder
    configs = [x for x in args.config]

    if folder is not None:
        configs.extend([f'{folder}/{x.name}' for x in list((CONFIG_FOLDER / folder).glob("*.json"))])

    run(configs)



