import seaborn as sns
import pandas as pd
import numpy as np
import math
import json
import matplotlib
import matplotlib.pyplot as plt

def LoadReward(path, policy, modelNum):
    # Gets the data frame, splits it up correctly
    data = None
    with open(path) as json_data:
        data = json.load(json_data)
    df = pd.DataFrame.from_dict(data['Data'])
    if (len(df) == 0):
        return []
    iteration = data['Iteration']
    success = data['Success']
    numHumans = data['NumHumans']
    steps = data['Steps']

    #  # Get's special values from the dataframe
    #  # Those that are the same for all entries
    #  iteration = df['Iteration'][0]
    #  hCount = df['NumHumans'][0]
    #  iterColumn = [iteration] * len(df2.index)
    #  timeColumn = [finalTime] * len(df2.index)
    #  policyColumn = [policy] * len(df2.index)
    #  policyColumn2 = [policy] * len(df.index)
    #  hColumn = [hCount] * len(df2.index)
    #  # Setup  Time Dataframe
    #  df['Policy'] = policyColumn2
    #  df2['Policy'] = policyColumn
    #  df2['Human Count'] = hColumn
    #  df2['Total Time'] = timeColumn
    #  df2['Trial'] = iterColumn
    w0 = 5.0
    w1 = -0.1
    w2 = -0.1
    bonus = 0
    df[df['DistScore'] < 0] *= 2.0
    if success:
        bonus = 100.0
    force = df['Force'].mean()
    blame = df['Blame'].mean()
    score = w0 * df['DistScore'] + w1 * df['Force'] + w2 * df['Blame']
    cumulative = (score.sum() + bonus) / len(df)
    episode = iteration
    return [policy, modelNum, episode, numHumans, success, steps, cumulative, force, blame]

def GetAllReward(path, policy, modelNum, start, end):
    data = []
    for i in range(start, end):
        #  print(i)
        filename = path + 'SocialGym' + str(i) + '.json'
        entry = LoadReward(filename, policy, modelNum)
        if (len(entry) != 0):
            data.append(entry)
    return data

def GetAllTrials(path, start, end, policy):
    data = []
    for i in range(start, end):
        fileStart = 1
        fileEnd = 100
        filename = path + str(i) + '/'
        results = GetAllReward(filename, policy, i, fileStart, fileEnd)
        pframe = pd.DataFrame(results, columns=['Policy', 'Iteration', 'Episode', 'Human Count', 'Success', 'Total Time', 'Score', 'Force', 'Blame'])
        print("-----Model Number: " + str(i))
        print("Average Timestep Reward: " + str(pframe["Score"].mean()))
        print("Success Rate: " + str(pframe["Success"].sum()))
        print("Average Force: " + str(pframe["Force"].mean()))
        print("Average Blame: " + str(pframe["Blame"].mean()))
        print("Average Time: " + str(pframe["Total Time"].mean()))
        print("---------------------------------")
        data = data + results
    return pd.DataFrame(data, columns=['Policy', 'Iteration', 'Episode', 'Human Count', 'Success', 'Total Time', 'Score', 'Force', 'Blame'])

#  df = GetAllReward('../../phase2/bc/1/', 'NicePPO', 21, 1000)
df = GetAllTrials('../../phase2/dqn/',  1, 16, 'DQN')
df.to_csv('DQN' + '_reward.csv')
