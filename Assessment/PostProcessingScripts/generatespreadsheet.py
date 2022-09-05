import os
from rich import print
import pandas as pd
import json

prestudyfolder = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\Pre-Study"
evalstudyfolder = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\Task_Data"
resultsfolder = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\Results"
# print("Updating Post-Processing Scripts with updated scripts from [blue]'Pre-Study\SubjectX\<Task>_0'\n")
n = 0
errs=0
idx = 0
open(os.path.join(resultsfolder,"Suturing.csv"), "w")

subjects = [s for s in os.listdir(prestudyfolder) if not s.endswith('.py') and not s.endswith('.bat')]
for s in subjects:
    tasks = [t for t in os.listdir(os.path.join(prestudyfolder, s)) if not t.endswith('.py') and not t.endswith('.bat')]
    for t in tasks:
        reps = [r for r in os.listdir(os.path.join(prestudyfolder, s, t)) if not r.endswith('.py') and not r.endswith('.bat')]
        for r in reps:
            print(os.path.join(prestudyfolder, s,t,r,r)+"_eval.json")
            with open(os.path.join(prestudyfolder, s,t,r,r)+"_eval.json") as jsonfile:
                d = pd.DataFrame(dict(json.load(jsonfile)),index = [idx])
                
                # print(list(d.keys()))
                # print(list(d.values()))
                # Loads csv to dataframe
                try:
                    taskdf= pd.read_csv(os.path.join(resultsfolder, t)+".csv",index_col=0)
                except:
                    taskdf = d
                    taskdf.to_csv(os.path.join(resultsfolder, t)+".csv",index_label="idx")
                    idx+=1
                    continue
                        
                taskdf = pd.concat([taskdf, d],)
                taskdf.to_csv(os.path.join(resultsfolder, t)+".csv",index_label="idx")
                print(taskdf)
                idx += 1

# subjects = [s for s in os.listdir(evalstudyfolder) if not s.endswith('.py') and not s.endswith('.bat')]
# for s in subjects:
#     tasks = [t for t in os.listdir(os.path.join(evalstudyfolder, s)) if not t.endswith('.py') and not t.endswith('.bat')]
#     for t in tasks:
#         reps = [r for r in os.listdir(os.path.join(evalstudyfolder, s, t)) if not r.endswith('.py') and not r.endswith('.bat')]
#         for r in reps:
#             print(os.path.join(evalstudyfolder, s,t,r,t)+"_post.bat")
#             generated = subprocess.call([os.path.join(evalstudyfolder, s,t,r,t)+"_post.bat"])
#             n+=1
#             if generated == 1: 
#                 print("[red]Error generating post-processing script for "+os.path.join(evalstudyfolder, s,t,r,t))
#                 errs+=1
                
