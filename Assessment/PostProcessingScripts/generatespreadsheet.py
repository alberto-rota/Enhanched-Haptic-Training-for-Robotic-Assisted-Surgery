import os
from rich import print
import pandas as pd
import json

prestudyfolder = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\Pre-Study"
evalstudyfolder = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\Evaluation"
resultsfolder = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\Results"
# print("Updating Post-Processing Scripts with updated scripts from [blue]'Pre-Study\SubjectX\<Task>_0'\n")
n = 0
errs=0
idx = 0

for t in ["Training1","Training2","Training3","Training4","LiverResection","Thymectomy","Nephrectomy","Suturing"]:
    if os.path.exists(os.path.join(resultsfolder, t)+".csv"):
        print("[yellow]Results file already exists for "+t+", creating a blank file")
        os.remove(os.path.join(resultsfolder, t)+".csv")

  
subjects = [s for s in os.listdir(evalstudyfolder) if not s.endswith('.py') and not s.endswith('.bat')]
for s in subjects:
    tasks = [t for t in os.listdir(os.path.join(evalstudyfolder, s)) if not t.endswith('.py') and not t.endswith('.bat')]
    for t in tasks:
        reps = [r for r in os.listdir(os.path.join(evalstudyfolder, s,t)) if not r.endswith('.py') and not r.endswith('.bat')]
        for r in reps:
            print(os.path.join(evalstudyfolder, s,t,r,r)+"_eval.json")
            with open(os.path.join(evalstudyfolder, s,t,r,r)+"_eval.json") as jsonfile:
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
                idx += 1
for t in ["Training1","Training2","Training3","Training4"]:
    print(pd.read_csv(os.path.join(resultsfolder, t)+".csv"))
    
# for t in ["LiverResection","Thymectomy","Nephrectomy","Suturing"]:
#     print(pd.read_csv(os.path.join(resultsfolder, t)+".csv"))

for t in ["Training1","Training2","Training3","Training4"]:
    print(t+" shape: ",end="")
    print(pd.read_csv(os.path.join(resultsfolder, t)+".csv").shape)

# for t in ["LiverResection","Thymectomy","Nephrectomy","Suturing"]:
#     print(t+" shape: ",end="")
#     print(pd.read_csv(os.path.join(resultsfolder, t)+".csv").shape)