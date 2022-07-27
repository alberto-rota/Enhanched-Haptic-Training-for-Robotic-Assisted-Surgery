# Active Constraints in Robot-Assisted Minimally Invasive Surgery
A surgical training simulator integrated with the daVinci surgical system, employing Active Constraints/Virtual Fixtures that provide a haptic force feedback during the surgical task under the assistance-as-needed paradigm.

The project, conducted in the Medical Robotics section of [NearLab](https://nearlab.polimi.it/), is Alberto's Master thesis concluding the MSc in Biomedical Engineering at _Politecnico di Milano_.

* **Supervisor:** Prof. De Momi Elena
* **Co-Supervisor:** Fan Ke
***
As the project is still undergoing development, this README is very synthetic and above all **INCOMPLETE**. Additional information and details are available by [contacting the author](mailto:alberto2.rota@mail.polimi.it)
***
_NOTE_: This repo currently only acts as a dev backup and a version control system for relevant files. **Downloading this repo will not download the simulator**, as larger files (blend, obj, packages, bin, ...) aren't stored here and will only be shared once the projecet is complete
***

## The project 
This work proposes a surgical simulator built in the Unity 3D environment where a virtual daVinci surgical robot is operated by a real teleoperation console. The user therefore sits at the console and grabs the MTMs: looking at the oculars of the HRSV system, he/she will see the feed from two virtual cameras placed in the virtual 3D scene, acquiring depth perception as a consequece

![unity](https://github.com/alberto-rota/MSc-Thesis-Active-Constraints-in-RAMIS/blob/main/readme/unity.png)

The main goal of the work is to implement and assess the efficacy of **virtual fixtures**: from the relative position of the surgical tool tip and the one of objects, obstacles, trajectories or guides, a "guidance force" can be computed in such a way that its application to the surgical tooltip will improve the surgical performance. The **"Assistance-as-needed"** paradigm is the main factor that is taken into consideration when deciding wether or not (and how) to apply such force 

As an example, this force will be computed so that it pushes the end-effector away from obstacles or, conversely, towards trajectories.

The force computed in the Unity environment is then, frame by frame, applied to the MTMs that the operator is holding as an **haptic force feedback**: transformations and changes in RFs are required. As a consequence, the user at the teleoperation console will feel an impedement or a guidance when performing the surgical tasks, allowing him/her to get optimally re-directed towards the desired pre-planned trajectory/workspace.


The image below is a very synthetic block diagram of the negative feedback for the haptic force

![diagram](https://github.com/alberto-rota/MSc-Thesis-Active-Constraints-in-RAMIS/blob/main/readme/diagram.png)

## The Tasks
6 Tasks are implemented by default on the simulator: 3 are surgical training tasks, while the others try to emulate a real surgical scenario.

![tasks](https://github.com/alberto-rota/MSc-Thesis-Active-Constraints-in-RAMIS/blob/main/readme/tasks.png)
