import os
import shutil

folder = os.path.dirname(os.path.realpath(__file__))
subjects = [s for s in os.listdir(folder) if not s.endswith('.py') and not s.endswith('.bat')]
for s in subjects:
    tasks = [f for f in os.listdir(os.path.join(folder, s)) if not f.endswith('.py') and not f.endswith('.bat')]
    print(s)
    for t in tasks:
        originalpostpy = [f for f in os.listdir(os.path.join(folder,s,t)) if f.endswith('.py')][0]
        originalpostbat = [f for f in os.listdir(os.path.join(folder,s,t)) if f.endswith('.bat')][0]
        originalgraphpy = [f for f in os.listdir(os.path.join(folder,s,t)) if f.endswith('.py')][1]
        originalgraphbat = [f for f in os.listdir(os.path.join(folder,s,t)) if f.endswith('.bat')][1]
        ntr = 0
        for tr in os.listdir(os.path.join(folder,s,t)):
            if not tr.endswith('.py') and not tr.endswith('.bat') and not tr == "SCENESTLS":
                shutil.copyfile(os.path.join(folder,s,t,originalpostpy), os.path.join(folder,s,t,tr,originalpostpy))
                shutil.copyfile(os.path.join(folder,s,t,originalpostbat), os.path.join(folder,s,t,tr,originalpostbat))
                shutil.copyfile(os.path.join(folder,s,t,originalgraphpy), os.path.join(folder,s,t,tr,originalgraphpy))
                shutil.copyfile(os.path.join(folder,s,t,originalgraphbat), os.path.join(folder,s,t,tr,originalgraphbat))
                ntr+=1
        print(f"> Updated {ntr} folders for {t}")
    print("\n")
print("\n")
