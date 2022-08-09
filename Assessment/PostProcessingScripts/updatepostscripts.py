import os
import shutil

taskdatafolder = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\Task_Data";
originalscriptsfolder = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\PostProcessingScripts";
subjects = [s for s in os.listdir(taskdatafolder) if not s.endswith('.py') and not s.endswith('.bat')]
for s in subjects:
    tasks = [f for f in os.listdir(os.path.join(taskdatafolder, s)) if not f.endswith('.py') and not f.endswith('.bat')]
    print(">> TASK DATA: "+s)
    for t in tasks:
        originalpostpy = [f for f in os.listdir(os.path.join(originalscriptsfolder,t)) if f.endswith('.py')][0]
        originalpostbat = [f for f in os.listdir(os.path.join(originalscriptsfolder,t)) if f.endswith('.bat')][0]
        originalgraphpy = [f for f in os.listdir(os.path.join(originalscriptsfolder,t)) if f.endswith('.py')][1]
        originalgraphbat = [f for f in os.listdir(os.path.join(originalscriptsfolder,t)) if f.endswith('.bat')][1]
        ntr = 0
        for tr in os.listdir(os.path.join(taskdatafolder,s,t)):
            if not tr.endswith('.py') and not tr.endswith('.bat') and not tr == "SCENESTLS":
                shutil.copyfile(os.path.join(originalscriptsfolder,t,originalpostpy), os.path.join(taskdatafolder,s,t,tr,originalpostpy))
                shutil.copyfile(os.path.join(originalscriptsfolder,t,originalpostbat), os.path.join(taskdatafolder,s,t,tr,originalpostbat))
                shutil.copyfile(os.path.join(originalscriptsfolder,t,originalgraphpy), os.path.join(taskdatafolder,s,t,tr,originalgraphpy))
                shutil.copyfile(os.path.join(originalscriptsfolder,t,originalgraphbat), os.path.join(taskdatafolder,s,t,tr,originalgraphbat))
                ntr+=1
        print(f"Updated {ntr} folders for {t}")
    print()
print("\n")

taskdatafolder = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\Pre-Study";
originalscriptsfolder = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\PostProcessingScripts";
subjects = [s for s in os.listdir(taskdatafolder) if not s.endswith('.py') and not s.endswith('.bat')]
for s in subjects:
    tasks = [f for f in os.listdir(os.path.join(taskdatafolder, s)) if not f.endswith('.py') and not f.endswith('.bat')]
    print(">> PRE-STUDY: "+s)
    for t in tasks:
        originalpostpy = [f for f in os.listdir(os.path.join(originalscriptsfolder,t)) if f.endswith('.py')][0]
        originalpostbat = [f for f in os.listdir(os.path.join(originalscriptsfolder,t)) if f.endswith('.bat')][0]
        originalgraphpy = [f for f in os.listdir(os.path.join(originalscriptsfolder,t)) if f.endswith('.py')][1]
        originalgraphbat = [f for f in os.listdir(os.path.join(originalscriptsfolder,t)) if f.endswith('.bat')][1]
        ntr = 0
        for tr in os.listdir(os.path.join(taskdatafolder,s,t)):
            if not tr.endswith('.py') and not tr.endswith('.bat') and not tr == "SCENESTLS":
                shutil.copyfile(os.path.join(originalscriptsfolder,t,originalpostpy), os.path.join(taskdatafolder,s,t,tr,originalpostpy))
                shutil.copyfile(os.path.join(originalscriptsfolder,t,originalpostbat), os.path.join(taskdatafolder,s,t,tr,originalpostbat))
                shutil.copyfile(os.path.join(originalscriptsfolder,t,originalgraphpy), os.path.join(taskdatafolder,s,t,tr,originalgraphpy))
                shutil.copyfile(os.path.join(originalscriptsfolder,t,originalgraphbat), os.path.join(taskdatafolder,s,t,tr,originalgraphbat))
                ntr+=1
        print(f"Updated {ntr} folders for {t}")
    print()
print("\n")
