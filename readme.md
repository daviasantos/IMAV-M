
# <img src="FIGURES\Logo.png" style="zoom:30%;" />

IMAV-M is a flight dynamics & control simulator of multirotor aerial vehicles (MAVs) developed for R&D in LRA (*Laboratório de Robótica Aérea*) at *Instituto Tecnológico de Aeronáutica* (Brazil). It was conceived to generate simulated motion variables and control effort commands for evaluating control, guidance, and navigation methods for eVTOL aircrafts and drones. It is written in script .m and can be executed and altered in both MATLAB or GNU Octave. 

The MAV physics as well as the flight control, guidance, and navigation algorithms are implemented in script .m, using the object-oriented paradigm. 

IMAV-M is an easy-to-use flight simulator intended to help people who are interested to learn or to do research and development on aerial robotics. 

Hope you enjoy and share it!

To learn about dynamic modeling, control, guidance, and navigation of MAVs, please visit [my homepage](https://www.professordavisantos.com/). You can find there the slides of my undergraduate and graduate courses, as well as my list publications. 



## How to use IMAV-M

First of all,  fork this repo into your GitHub profile and then download or clone it on your computer. It has one main folder named `MATLAB`, where you can find all the simulator source files.  

The `MATLAB` folder has three sub-folders: `Classes`, `Conversions`, and `Plots`. The sub-folder `Classes` contains one class for each functionality of the simulator (*e.g.*, `CCOntrol` is a class which implements the flight control laws). The sub-folder `Conversion` contains basic math functions for coordinate conversions, signal saturation, and construction of a skew-symmetric matrix. Finally, `Plots` contains the scripts for generating the graphics of the simulated variables.    

In the repository root you can find the main script named `IMAVM.m` and a data file called `Parameters.mat`, which contains all the simulation default parameters. 

Optionally, you can change the default parameters of `Parameters.mat`. For this end, you can use the IMAVM MATLAB app. The installer for this app is also available in `MATLAB`. To install the IMAVM app, just click on `IMAVM.mlappinstall` from MATLAB *current directory*. 

Finally, to execute the simulator, I suggest using MATLAB 2019b (or later) or GNU Octave 5.2.0 (or later).  

**Steps to run IMAV-M:**

1. (Optional) Run the IMAVM app and make the desired modifications on the parameter set. If you are using GNU Octave, you will need to edit the `.mat` data file manually.
2. From the MATLAB command line, run `~/MATLAB/IMAVM.m`. It will show some basic info on the screen. You can just press ENTER to start the simulation.

After finishing the simulation, the execution time will be shown in the command line and some graphics will be automatically generated. 

## Comments about the implementation

The MATLAB part of IMAV-M is implemented using the object orientation paradigm. Here you can see the list of classes developed for this project:

* `CMav`: Includes the MAV parameters, input and output variables, and the functions to compute resulting torque and force, propellers' thrusts, and integration of the equations of motion.

* `CSensors`: Implements the models of the navigation sensor's measurements. 

* `CUncer`: Implements the force and torque disturbances, as well as magnetic interference. 

* `CControl`: Includes the flight controller parameters, input and output variables, and the functions to compute the attitude control, the position control, the control allocation, the reference filter, and the 3D attitude command.

* `CGuidance`: Implements waypoint-based guidance as an external law over the MAV in closed loop with position and attitude controllers. 
  
* `CNavigation`: Implements an attitude determination extended Kalman filter (EKF) and process the GPS measurements to estimate 3D position and velocity. 



## Acknowledgment

I would like to thank a lot all my students and colleagues who somehow helped me to start this project. Many of the ideas and methods implemented in IMAV-M have been discussed in the graduate courses [MP-282 Dynamic Modeling and Control of Multirotor Aerial Vehicles](https://www.professordavisantos.com/modeling-control-mav/) and [MP-208 Optimal Filtering with Aerospace Applications](https://www.professordavisantos.com/category/courses/mp-208/), which I have offered since 2014 in the Aeronautics and Mechanics Graduate Program at [ITA/Brazil](https://www.ita.br). I am also grateful to the research agencies which supported this project: FAPESP (grant 2019/05334-0) and CNPq (302637/2018-4).