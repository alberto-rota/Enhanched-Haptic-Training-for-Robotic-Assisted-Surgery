import os
import subprocess
import shutil
from rich import print

folder = os.path.dirname(os.path.realpath(__file__))
s = "SubjectX"
tasks = [f for f in os.listdir(os.path.join(folder, s)) if not f.endswith('.py') and not f.endswith('.bat')]
nevals = []
for t in tasks:
    ntr = 0
    for tr in os.listdir(os.path.join(folder,s,t)):
        if not tr.endswith('.py') and not tr.endswith('.bat') and not tr.endswith('.json') and not tr == "SCENESTLS":
            print("Removing [red]"+os.path.join(folder,s,t,tr)+"[/red]")
            shutil.rmtree(os.path.join(folder,s,t,tr))
print()