import os
import shutil
from rich import print
import subprocess

# prestudyfolder = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\Pre-Study"
evalstudyfolder = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\Evaluation"

# print("Updating Post-Processing Scripts with updated scripts from [blue]'Pre-Study\SubjectX\<Task>_0'\n")
n = 0
errs=0
nexisting = 0
                
subjects = [s for s in os.listdir(evalstudyfolder) if not s.endswith('.py') and not s.endswith('.bat')]
for s in subjects:
    tasks = [t for t in os.listdir(os.path.join(evalstudyfolder, s)) if not t.endswith('.py') and not t.endswith('.bat')]
    for t in tasks:
        reps = [r for r in os.listdir(os.path.join(evalstudyfolder, s, t)) if not r.endswith('.py') and not r.endswith('.bat')]
        for r in reps:
            if os.path.exists(os.path.join(evalstudyfolder, s,t,r,r)+"_eval.json"):
                nexisting+=1
                # print("[yellow]Evaluation file already exists for "+os.path.join(evalstudyfolder, s,t,r,t))
                continue
                
            print(os.path.join(evalstudyfolder, s,t,r,t)+"_post.bat")
            generated = subprocess.call([os.path.join(evalstudyfolder, s,t,r,t)+"_post.bat"])
            n+=1
            if generated == 1: 
                print("[red]Error generating post-processing script for "+os.path.join(evalstudyfolder, s,t,r,t))
                errs+=1
            else:
                print("[green]Correctly Generated post-processing script for "+os.path.join(evalstudyfolder, s,t,r,t))
                
print(f"{errs} errors occured while trying to generate {n} json evals")
print(f"{nexisting} json evals already existed")
