import os
import subprocess
import shutil

folder = os.path.dirname(os.path.realpath(__file__))
tasks = [f for f in os.listdir(folder) if not f.endswith('.py') and not f.endswith('.bat')]
nevals = []
for t in tasks:
    ntr = 0
    for tr in os.listdir(os.path.join(folder, t)):
        if not tr.endswith('.py') and not tr.endswith('.bat') and not tr == "SCENESTLS":
            postscript = [f for f in os.listdir(os.path.join(folder, t, tr)) if f.endswith('_post.bat')][0]
            # os.chdir(os.path.join(folder, t, tr))
            print(os.path.join(folder,t,tr, postscript))
            os.system(os.path.join(folder,t,tr, postscript))
            ntr+=1
    nevals.append(ntr)
for nt,t in enumerate(tasks):
    print(f"> Generated {nevals[nt]} eval JSONs for task: {t}")