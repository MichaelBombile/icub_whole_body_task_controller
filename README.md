# Humanoid whole Body Tasks Controllers

This repository conatains a set of controllers for whole body manipulation taks with anticipatory postural ajustments on the humanoid robot iCub. Regarding the tasks specification, it contains at the manipulation level:
- a Dynamical Systems-based bimanual coordinated motion controller
- a Bimanual coordinated force controller
Regarding Balance, it contains:
- a Center of mass (CoM) position regulator 
- Stepping controller
Theses controllers are complemented with a feedforward balance controller that generates anticipatory actions in form of change of the CoM position, feet position, or internal motion-based body posture through according to an expected balance perturbation associated with the robot interaction.
At the task execution level, it contains:
- a whole body inverse dynamic controller that computes torques commands
- a whole body inverse kinematic controller that computes joints velocity and position commands


---
## System Requirements
Ubuntu >= 16.04 and Gazebo >=7

### Dependencies
Prior to the compilation of this controller, make sure you have installed the following software:
- [YARP](https://github.com/robotology/yarp)
- [ICUB](https://github.com/robotology/icub-main) (Install from sources!)
- [Eigen3](): Eigen 3 version >=3.2.9 
- [qpOASES](https://projects.coin-or.org/qpOASES/wiki/QpoasesInstallation)
- [iDynTree](https://github.com/robotology/idyntree) 
All these main dependencies can be better installed through the installation of [robotology-superbuild](https://github.com/robotology/robotology-superbuild).

For simulation
- [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
- [gazebo_yarp_plugins](https://github.com/robotology/gazebo-yarp-plugins)
- [icub-gazebo](https://github.com/robotology/icub-gazebo)

**Easiest option (on clean installation):**
1. Install latest version of Eigen3 as above.
2. Install GazeboX and libgazeboX-dev (X >=7): [installation instructions](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install) 
3. Install all yarp/iCub/gazebo-plugin libraries with [robotology-superbuild](https://github.com/robotology/robotology-superbuild)

## Compilation and build

Clone the repository

```bash
$ git clone https://MichaelBombile/icub_whole_body_task_controller.git
```

### build
Edit first the `CMakeLists.txt` file to indicate :
- the correct paths of qpOASES lib file (`libqpOASES.so`) and qpOASES directory

Once the CMakeList.txt edited, the controller can be built. Just run

```bash
$ cd ~/icub_whole_body_task_controller
$ mkdir build && cd build
$ cmake .. && make
```
This will create the ``./icub_whole_body_task_controller`` executable which runs the controller. It will be placed in ``~/icub_whole_body_task_controller/build``.

---

## Running the controller
Running this controller in its current version is still quite elaborate. 

- start yarpserver, in one terminal type the following
```
$ yarpserver
```
- (simulation) start gazebo simulator and import include the robot model (`iCub (no hands)`)
```
$ cd ~/robotology-superbuild/robotology
$ gazebo ./icub-gazebo/worlds/icub.world
```

- Bring the robot in home position 
```
$ yarpmotorgui --from homePoseBalancing.ini --robot robot_name 
```
robot_name: (e.g. icub or icubSim) and then press the 'Home All' button

- Launch the controller as follows : 
```
$ ./WalkingGrasping --from ../config/BalanceWalkingController.ini
```
This code will run the controller with a constant velocity (defined in the config file below) for a predetermined duration (in seconds, also defined in the config file).

#### Known Run-time Issues (and solutions)
- If you get the error `did not find model.urdf` you must replace, `model.urdf` in the `BalanceWalkingController.ini` config file with `~/robotology-superbuild/build/install/share/codyco/robots/icubGazeboSim/model.urdf`

---

## Expected behavior

The behavior of the controller is  demonstrated [here](https://www.youtube.com/watch?v=9hKOVHDDnfc&t=16s)

---

## Controller details


### Configuration file

The configuration file is `BalanceWalkingController.ini` locate in the `config` folder. The parameters are

- `robot`: name of the robot to connect to (e.g icubSim for simulation and icub for the real robot)
- `name`: name of the module. All ports open by the controller will include this name (default: walkingGrasping)
- `period`: period of the controller thread (default 40 ms)
- `modulePeriod`: period of the module
- `duration`: running time duration in secondes, once reached the robot stops and goes back to its initial standing posture

- `FT_feedback`: mode of reactive walking (0: direct velocity mode with no force interaction; 1: admittance control using feet forces/moments and 2: admittance using measured arms forces/momemts)

- `VelocityX`: the initial forward or backward velocity of the robot in [m/s]
- `VelocityY`:  the initial lateral velocity of the robot in [m/s]
- `OmegaZ`:	the initial rotational velocity of the robot about the vertical axis in [m/s]	

```
Note: should you want to change the velocity at run time, you can do it in the while loop of the main.cpp file. 
An input port will be added for that purpose soon.
``` 


### Other parameters

Others parameters are in the `src/ControllerParameters.cpp` file. 



#### Citing this contribution
In case you want to cite the content of this controller implementation, please refer to [Capture-point based balance and reactive omnidirectional walking controller](https://ieeexplore.ieee.org/document/8239532) and use the following bibtex entry:

``` 
 @inproceedings{bombile2017capture,
  title={Capture-point based balance and reactive omnidirectional walking controller},
  author={Bombile, Michael and Billard, Aude},
  booktitle={Humanoid Robotics (Humanoids), 2017 IEEE-RAS 17th International Conference on},
  pages={17--24},
  year={2017},
  organization={IEEE}
}
```


