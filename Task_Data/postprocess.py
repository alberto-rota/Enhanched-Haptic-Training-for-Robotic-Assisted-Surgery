# %%
import os
import pandas as pd
import json
from rich import print

df = pd.DataFrame()
folder = os.path.dirname(os.path.realpath(__file__))
subjects = [s for s in os.listdir(folder) if not s.endswith('.py') and not s.endswith('.bat')]
for s in subjects:
    tasks = [f for f in os.listdir(os.path.join(folder, s)) if not f.endswith('.py') and not f.endswith('.bat')]
    for t in tasks:
        ntr = 0
        for tr in os.listdir(os.path.join(folder,s,t)):
            if not tr.endswith('.py') and not tr.endswith('.bat') and not tr.endswith('.json') and not tr == "SCENESTLS":
                jsoneval = [f for f in os.listdir(os.path.join(folder,s,t,tr)) if f.endswith('_eval.json')][0]
                with open(os.path.join(folder,s,t,tr, jsoneval)) as json_file:
                    eval = json.load(json_file)
                    df = df.append(eval, ignore_index=True)
print(df)
df.to_csv("C:\\Users\\alber\\Desktop\\evalbigchungus.csv")