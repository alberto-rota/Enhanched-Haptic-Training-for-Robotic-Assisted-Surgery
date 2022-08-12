This page will guide you through the initialization and setup of the **dVRK** in the [NearLab](https://nearlab.polimi.it/medical/) at Building 7 of *Politecnico di Milano*.

The lab features 3 computers: **GRU**, **dVRK** and **Win**. Using the daVinci surgical robot requires full access (log-in credentials are necessary) to at least GRU and dVRK.

***

## GRU Computer
The GRU station is directly connected to the dVRK and must be the ROS master. 

**roscore** must be running on this machine.
Open a terminal with `ctrl+alt+T` and type:
```
roscore
```
***
## dVRK Computer
The dVRK station handles the teleoperation and hosts the GUI of the dVRK system. 

It must run a `rosbridge server` communicating with the _rosmaster_ of the GRU station. To do this, open a terminal and run the alias:
```
roslaunchserver
```
**MAKE SURE** that the command logs the ROS_MASTER_URI variable with the IP address of the GRU station: *192.168.1.1* or *GRU.local*

In order to open the teleoperation console GUI, open a terminal and run:
```
dvrk_simulator
```
This is a bash script that sources the ROS overlay, closes the safety relays and launches the teleoperation console. 

**MAKE SURE** that the relays are closed correctly by checking the log of the command. If this is not the case:
* Close the terminal and the console
* Switch the firewire cable to the adjacent port
* Run the command again
***

## Win Computer
When working on a Unity environment that shall read ROS topics from the dVRK system:
* **RosSharp** must be a package imported into your project
* Unity GameObjects subscribing to ROS topics must have a *RosConnector* component that links **to the IP address of the machine where the *rosbridge server* is running**. If, as this guide suggests, the *rosbridge server* has been launched on the dVRK station, use its IP **192.168.1.2**. In case the *rosbridge server* has been launched on the GRU machine, the IP that shall be used is **192.168.1.1**
***

## Troubleshooting
* IPs of the computers in the NearLab are static, hence they **should not** change by themselves. Check the correct IP of the machine of interest by running `ifconfig` (Linux) or `ipconfig` (Windows). IPs of a local ethernet connection always start with **192.168**
* A very detailed documentation of the dVRK is provided at the [Wiki](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki) of the official repo from JHU

