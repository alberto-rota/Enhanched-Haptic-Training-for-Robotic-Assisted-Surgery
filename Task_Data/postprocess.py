import os
import subprocess
import shutil
import json

folder = os.path.dirname(os.path.realpath(__file__))
tasks = [f for f in os.listdir(folder) if not f.endswith('.py') and not f.endswith('.bat')]
for t in tasks:
    ntr = 0
    for tr in os.listdir(os.path.join(folder, t)):
        if not tr.endswith('.py') and not tr.endswith('.bat') and not tr == "SCENESTLS":
            jsoneval = [f for f in os.listdir(os.path.join(folder, t, tr)) if f.endswith('_eval.json')][0]
            with open(os.path.join(folder,t,tr, jsoneval)) as json_file:
                eval = json.load(json_file)
                print(eval)