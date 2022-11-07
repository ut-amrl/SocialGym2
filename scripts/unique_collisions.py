from tensorboard.backend.event_processing import event_accumulator

from pathlib import Path

RUN_NAME = '5a_eo_collision_ender_p3__eval'

TENSORBOARD_FOLDER = Path(__file__).parent.parent / f'data/{RUN_NAME}'
ENVINFO_TENSORBOARD_FOLDER = TENSORBOARD_FOLDER / 'env_info/number_of_collisions'
REWARDS_TENSORBOARD_FOLDER = TENSORBOARD_FOLDER / 'rewards/scalars/collisions'

file = None

for f in REWARDS_TENSORBOARD_FOLDER.glob('*tfevents*'):
    file = f

file2 = None
for f in ENVINFO_TENSORBOARD_FOLDER.glob('*tfevents*'):
    file2 = f

ea_env = event_accumulator.EventAccumulator(str(file2), size_guidance={
    event_accumulator.SCALARS: 0
})

ea = event_accumulator.EventAccumulator(str(file), size_guidance={
    event_accumulator.SCALARS: 0
})

ea_env.Reload()
ea.Reload()

print(ea.Tags())

evts = ea.Scalars('rewards/scalars')
cols_per_step = [int(abs(x.value) / 0.1) for x in ea.Scalars('rewards/scalars')]

cur_num_collisions = 0
unique_collisions = 0

for c in cols_per_step:
    if cur_num_collisions == c:
        continue
    elif cur_num_collisions > c:
        cur_num_collisions = c
    else:
        unique_collisions += c - cur_num_collisions
        cur_num_collisions = c

episodes = ea_env.Scalars('env_info')
total_episodes = sum([1 for _ in episodes])

print(unique_collisions / total_episodes)


