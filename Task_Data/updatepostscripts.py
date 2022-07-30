import os
import shutil

folder = os.path.dirname(os.path.realpath(__file__))
tasks = [f for f in os.listdir(folder) if not f.endswith('.py') and not f.endswith('.bat')]
for t in tasks:
    trials = []
    originalpostpy = [f for f in os.listdir(os.path.join(folder, t)) if f.endswith('.py')][0]
    originalpostbat = [f for f in os.listdir(os.path.join(folder, t)) if f.endswith('.bat')][0]
    originalgraphpy = [f for f in os.listdir(os.path.join(folder, t)) if f.endswith('.py')][1]
    originalgraphbat = [f for f in os.listdir(os.path.join(folder, t)) if f.endswith('.bat')][1]
    ntr = 0
    for tr in os.listdir(os.path.join(folder, t)):
        if not tr.endswith('.py') and not tr.endswith('.bat') and not tr == "SCENESTLS":
            shutil.copyfile(os.path.join(folder,t,originalpostpy), os.path.join(folder,t,tr,originalpostpy))
            shutil.copyfile(os.path.join(folder,t,originalpostbat), os.path.join(folder,t,tr,originalpostbat))
            shutil.copyfile(os.path.join(folder,t,originalgraphpy), os.path.join(folder,t,tr,originalgraphpy))
            shutil.copyfile(os.path.join(folder,t,originalgraphbat), os.path.join(folder,t,tr,originalgraphbat))
            ntr+=1
    print(f"> Updated {ntr} folders for {t}")
print("\n")
