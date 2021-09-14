import seaborn as sns
import pandas as pd
import numpy as np
import math
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
import matplotlib.ticker as ticker
from pympler import asizeof
import gc
import json

matplotlib.use("pgf")
matplotlib.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
})
#  %matplotlib inline
import re
from os import listdir
from os.path import isfile, join

def LoadEval(path, policy):
    #  df_full = pd.read_json(path)
    #  df2 = df_full
    #  df2 = pd.read_json(df2['Data'].to_json()).transpose()
    #  df = df_full.drop(columns=['Data'])
    #  df = df.drop(columns=['Demos'])
    data = None
    with open(path) as json_data:
        data = json.load(json_data)
    df2 = pd.DataFrame.from_dict(data['Data'])
    df = pd.DataFrame.from_dict(data)
    df = df.drop(columns=['Data'])
    df = df.drop(columns=['Demos'])
    #print(df)
    df = df.head(1)
    #  df_full = None
    iteration = df['Iteration'][0]
    finalTime = df['Steps'][0]
    hCount = df['NumHumans'][0]
    iterColumn = [iteration] * len(df2.index)
    timeColumn = [finalTime] * len(df2.index)
    policyColumn = [policy] * len(df2.index)
    policyColumn2 = [policy] * len(df.index)
    hColumn = [hCount] * len(df2.index)
    df2['Policy'] = policyColumn
    df2['Human Count'] = hColumn
    df2['Total Time'] = timeColumn
    df2['Trial'] = iterColumn
    df['Policy'] = policyColumn2
    return df, df2

def CollectData(path, policy):
    data = None
    data2 = None
    start = True
    iterStart =  3
    iterEnd = 1999
    for i in range(iterStart, iterEnd):
        print(i)
        filename = path + 'SocialGym' + str(i) + '.json'
        df,df2 = LoadEval(filename, policy)
        if (start):
            data = df
            data2 = df2
            start = False
        else:
            data = pd.concat([data, df], axis=0)
            data2 = pd.concat([data2, df2], axis=0)
        df = None
        df2 = None
    return data,data2

path = './phase3/ga/'
policy = 'GoAlone'
fullTime, fullData = CollectData(path, policy)
fullTime = fullTime.rename(columns={'Iteration': 'Trial', 'Steps': 'Final Time', 'NumHumans' : 'Human Count'})
fullTime.to_csv(policy + "_gym_time.csv")
fullData.to_csv(policy + "_gym_metrics.csv")
print("Conversion Complete")
