# Virtual Fixtures in Robot-Assisted Minimally Invasive Surgery
A surgical training simulator integrated with the daVinci surgical system, employing Active Constraints/Virtual Fixtures that provide a haptic force feedback during the surgical task under the assistance-as-needed paradigm.

The project, conducted in the Medical Robotics section of [NearLab](https://nearlab.polimi.it/), is Alberto's Master thesis concluding the MSc in Biomedical Engineering at _Politecnico di Milano_.

* **Supervisor:** Prof. De Momi Elena
* **Co-Supervisor:** Fan Ke
***
_As the project is still undergoing development, this README is very synthetic and above all **INCOMPLETE**. Additional information and details are available by [contacting the author](mailto:alberto2.rota@mail.polimi.it)_


_NOTE_: This repo currently only acts as a dev backup and a version control system for relevant files. **Downloading this repo will not download the simulator**, as larger files (blend, obj, packages, bin, ...) aren't stored here and will only be shared once the project is complete
***

## The project 
This work proposes a surgical simulator built in the Unity 3D environment where a virtual daVinci surgical robot is operated by a real teleoperation console. The user therefore sits at the console and grabs the MTMs: looking at the oculars of the HRSV system, he/she will see the feed from two virtual cameras placed in the virtual 3D scene, acquiring depth perception as a consequece

![unity](https://github.com/alberto-rota/Virtual-Fixtures-in-Robotic-Assisted-Surgery/blob/main/Notes/readme/unity.png)

The main goal of the work is to implement and assess the efficacy of **virtual fixtures**: from the relative position of the surgical tool tip and the one of objects, obstacles, trajectories or guides, a "guidance force" can be computed in such a way that its application to the surgical tooltip will improve the surgical performance. The **"Assistance-as-needed"** paradigm is the main factor that is taken into consideration when deciding wether or not (and how) to apply such force 

As an example, this force will be computed so that it pushes the end-effector away from obstacles or, conversely, towards trajectories.

#### Experimental Study
An experimental study will assess the effectiveness, utility and clinical relevance of Virtual Fixtures. In this phase, a population of volunteer subjects will be divided in:
* An **Assisted Group** (group **A**), where the haptic force feedback will be provided when performing surgical training tasks on the simulator
* A **Control Group** (group **C**), to whom no force feedback will be delivered  

The results of the study are anticipated as follows:
1. Subjects in **A** will learn surgical skills more rapidly than subjects in **C**
2. Subjects in **A** will reach an overall better performance than subjects in **C**, at the end of the study
3. Subjects in **A** will better skill retention 
4. Sbjects in **A** will show better skill transfer

## Framework
The trainee sits at the teleoperation console of a *daVinci* surgical robot and manipulates the MTMs

His motion will be replicated inside of the virtual surgical environment on virtual PSMs

In the virtual surgical scene, the relative position of the surgical tool and the anatomical structures will be used to compute the haptic feedback force

The force computed in the Unity environment is then, frame by frame, applied to the MTMs that the operator is holding as an **haptic force feedback**: transformations and changes in RFs are required. As a consequence, the user at the teleoperation console will feel an impediment or a guidance when performing the surgical tasks, allowing him/her to get optimally re-directed towards the desired pre-planned trajectory/workspace.

The image below is a very synthetic block diagram of the negative feedback for the haptic force

![diagram](https://github.com/alberto-rota/Virtual-Fixtures-in-Robotic-Assisted-Surgery/blob/main/Notes/readme/diagram.png)

# The Surgical Tasks
8 Tasks are implemented by default on the simulator: 4 are surgical training tasks, while the others try to emulate a real surgical scenario.

Here is a preview of some of them.

![tasks](https://github.com/alberto-rota/Virtual-Fixtures-in-Robotic-Assisted-Surgery/blob/main/Notes/readme/taskscollage.gif)
