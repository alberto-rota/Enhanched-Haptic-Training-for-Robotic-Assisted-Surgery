import os
import shutil
from rich import print

originalto = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\PostProcessingScripts";
updatefrom = "C:\\Users\\alber\\Desktop\\Active_Constraints\\Assessment\\Pre-Study\\SubjectX";

print("Updating Post-Processing Scripts with updated scripts from [blue]'Pre-Study\SubjectX\<Task>_0'\n")

tasks = [f for f in os.listdir(os.path.join(updatefrom, )) if not f.endswith('.py') and not f.endswith('.bat')]
for t in tasks:
    updatedpostpy = os.path.join(updatefrom,t,t+"_0",t+"_post.py")
    updatedpostbat = os.path.join(updatefrom,t,t+"_0",t+"_post.bat")
    updatedgraphpy = os.path.join(updatefrom,t,t+"_0",t+"_graph.py")
    updatedgraphbat = os.path.join(updatefrom,t,t+"_0",t+"_graph.bat")
    originalpostpy = os.path.join(originalto,t,t+"_post.py")
    originalpostbat = os.path.join(originalto,t,t+"_post.bat")
    originalgraphpy = os.path.join(originalto,t,t+"_graph.py")
    originalgraphbat = os.path.join(originalto,t,t+"_graph.bat")
    
    shutil.copyfile(updatedpostpy, originalpostpy)
    shutil.copyfile(updatedpostbat, originalpostbat)
    shutil.copyfile(updatedgraphpy, originalgraphpy)
    shutil.copyfile(updatedgraphbat, originalgraphbat)
    
    print("[yellow]"+updatedpostpy)
    print("   [green]>>>   "+originalpostpy)
    print("[yellow]"+updatedpostbat)
    print("   [green]>>>   "+originalpostbat)
    print("[yellow]"+updatedgraphpy)
    print("   [green]>>>   "+originalgraphpy)
    print("[yellow]"+updatedgraphbat)
    print("   [green]>>>   "+originalgraphbat)
