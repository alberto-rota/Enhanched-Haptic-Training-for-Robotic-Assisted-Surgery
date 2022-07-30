import os
import subprocess
import shutil
from rich import print
folder = os.path.dirname(os.path.realpath(__file__))
subjects = [s for s in os.listdir(folder) if not s.endswith('.py') and not s.endswith('.bat')]
for s in subjects:
    tasks = [f for f in os.listdir(os.path.join(folder, s)) if not f.endswith('.py') and not f.endswith('.bat')]
    print(s)
    nevals = []
    for t in tasks:
        ntr = 0
        for tr in os.listdir(os.path.join(folder,s,t)):
            if not tr.endswith('.py') and not tr.endswith('.bat') and not tr.endswith('.json') and not tr == "SCENESTLS":
                postscript = [f for f in os.listdir(os.path.join(folder,s,t,tr)) if f.endswith('_post.bat')][0]
                print(os.path.join(folder,s,t,tr, postscript))
                os.system(os.path.join(folder,s,t,tr, postscript))
                ntr+=1
        nevals.append(ntr)
        
    print(s)
    for nt,t in enumerate(tasks):
        print(f"> Generated {nevals[nt]} eval JSONs for task: {t}")
    print("\n")
print("\n")