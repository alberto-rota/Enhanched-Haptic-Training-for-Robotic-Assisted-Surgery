import os
import shutil

folder = os.path.dirname(os.path.realpath(__file__))
tasks = [f for f in os.listdir(folder) if not f.endswith('.py') and not f.endswith('.bat')]
for t in tasks:
    trials = []
    originalpy = [f for f in os.listdir(os.path.join(folder, t)) if f.endswith('.py')][0]
    originalbat = [f for f in os.listdir(os.path.join(folder, t)) if f.endswith('.bat')][0]
    for tr in os.listdir(os.path.join(folder, t)):
        if not tr.endswith('.py') and not tr.endswith('.bat') and not tr == "SCENESTLS":
            shutil.copyfile(os.path.join(folder,t,originalpy), os.path.join(folder,t,tr,originalpy))
            shutil.copyfile(os.path.join(folder,t,originalbat), os.path.join(folder,t,tr,originalbat))
            print(f"Updated post processig scripts in{os.path.join(folder,t,tr)}")
print()