import os
from rich import print
import subprocess

subjectA = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\Evaluation\\SubjectA"
subjectV = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\Evaluation\\SubjectD"


  
for t in  ["Training1","Training2","Training3","Training4"]:
    print(" > "+os.path.join(subjectV,t,t+"_11",t)+"_graph.bat")
    try:
        graphproc = subprocess.call([os.path.join(subjectV,t,t+"_11",t)+"_graph.bat"])
    except:
        pass

for t in  ["LiverResection","Thymectomy","Nephrectomy","Suturing"]:
    print(" > "+os.path.join(subjectV,t,t+"_2",t)+"_graph.bat")
    try:
        graphproc = subprocess.call([os.path.join(subjectV,t,t+"_2",t)+"_graph.bat"])
    except:
        pass
