import os
import subprocess
from pathlib import Path
import shutil


SCRIPTS_FOLDER = Path(__file__).parent.absolute()
ROOT_FOLDER = SCRIPTS_FOLDER.parent
VECTOR_DISPLAY_FOLDER = ROOT_FOLDER / 'docker/vectordisplay'
MAPS_FOLDER = VECTOR_DISPLAY_FOLDER / 'maps'
SRC_FOLDER = ROOT_FOLDER / 'src'
TEMPLATES_FOLDER = SRC_FOLDER / 'templates'
SUBMODULES_FOLDER = ROOT_FOLDER / 'submodules'
AMRL_MAPS_FOLDER = SUBMODULES_FOLDER / 'amrl_maps'
UT_MULTI_ROBOT_SIM_FOLDER = SUBMODULES_FOLDER / 'ut_multirobot_sim'
UT_MULTI_ROBOT_SIM_MAPS_FOLDER = UT_MULTI_ROBOT_SIM_FOLDER / 'maps'


def step_1_setup_git_branch():
    print("STEP 1: updating git submodules...")
    out = subprocess.Popen('git submodule update --init --recursive'.split(), stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out.wait()

    out = subprocess.Popen('pip install -r requirements.txt'.split(), stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out.wait()


def step_2_move_map_files():
    print("STEP 2: moving files around...")
    environment_path = TEMPLATES_FOLDER / 'exp1/train/easy'
    environment_map_path = MAPS_FOLDER / 'exp1/train/easy'

    vectormap_file = environment_map_path / f'{environment_path.name}.vectormap.txt'
    vectormap_json_file = environment_path / f'{environment_path.name}.vectormap.json'
    navigation_file = environment_map_path / f'{environment_path.name}.navigation.txt'
    navigation_json_file = environment_map_path / f'{environment_path.name}.navigation.json'

    amrl_folder = AMRL_MAPS_FOLDER / 'exp1/train/easy'
    amrl_folder.mkdir(exist_ok=True, parents=True)

    (amrl_folder / f'{environment_path.name}.vectormap.txt').parent.mkdir(exist_ok=True, parents=True)

    shutil.copyfile(vectormap_file, amrl_folder / f'{environment_path.name}.vectormap.txt')
    shutil.copyfile(vectormap_json_file, amrl_folder / f'{environment_path.name}.vectormap.json')
    shutil.copyfile(navigation_file, amrl_folder / f'{environment_path.name}.navigation.txt')
    shutil.copyfile(navigation_json_file, amrl_folder / f'{environment_path.name}.navigation.json')

    sub_amrl_folder = (amrl_folder / 'exp1/train/easy').parent
    sub_amrl_folder.mkdir(exist_ok=True, parents=True)

    shutil.copyfile(vectormap_file, sub_amrl_folder / f'{environment_path.name}.vectormap.txt')
    shutil.copyfile(vectormap_json_file, sub_amrl_folder / f'{environment_path.name}.vectormap.json')
    shutil.copyfile(navigation_file, sub_amrl_folder / f'{environment_path.name}.navigation.txt')
    shutil.copyfile(navigation_json_file, sub_amrl_folder / f'{environment_path.name}.navigation.json')

    utmulti_folder = UT_MULTI_ROBOT_SIM_MAPS_FOLDER / 'exp1/train/easy'
    utmulti_folder.mkdir(exist_ok=True, parents=True)
    (utmulti_folder / f'exp1/train/easy.vectormap.txt').parent.mkdir(exist_ok=True, parents=True)

    shutil.copyfile(vectormap_file, utmulti_folder / f'{environment_path.name}.vectormap.txt')
    shutil.copyfile(vectormap_json_file, utmulti_folder / f'{environment_path.name}.vectormap.json')
    shutil.copyfile(navigation_file, utmulti_folder / f'{environment_path.name}.navigation.txt')
    shutil.copyfile(navigation_json_file, utmulti_folder / f'{environment_path.name}.navigation.json')


def step_3_build_docker():
    print("STEP 3: Building docker.  WARNING: Can take 30 minutes! (logs saved in ./tmp.log, tail it via 'tail -f ./tmp.log')")
    if Path('./tmp.log').exists():
        os.remove('./tmp.log')

    f = open('./tmp.log', 'w')
    out = subprocess.Popen('./config_runner/build.sh > ./tmp.log'.split(), stdin=subprocess.PIPE, stdout=f, stderr=f)
    out.wait()


if __name__ == "__main__":
    step_1_setup_git_branch()
    step_2_move_map_files()
    step_3_build_docker()
