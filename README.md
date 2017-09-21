# Adversary Control

## Overview

Simulation of adversary control in a linear, 2-dimensional discrete system in state-space representation (soon to be expanded to higher dimensional systems) .

We assume that the system operates under linear constraints regarding both the input of the controllers and regarding its state. Depending on the type of scenario (see below) a specific area is defined, called the desired operation area, within which the system wants to operate.

The system is controlled by two opposing parties : The contractive and the expanding controllers.
The objective of the contractive controller is to keep the system operating in a stable equilibrium point within the desired operation area , while the objective of the expanding controller is to disrupt this stability and force it to operate outside
of it (while respecting input constraints).

The following scenarios of adversary control are explored :

*Regular* : The desired operation area is defined as the intersection of the input constraints and the state constraints. The two controllers take turns affecting the system for a preset amount of time steps.

*Covert* : In the case of covert adversary control, two areas are defined : the alarm constraints area and the desired operation area (the second being a subset of the first). The objectives of the expanding controller in this case, is to force the system to a equilibrium point outside the desired operation area while respecting the alarm constraints, and with the minimum amount of time spent controlling the system.


## Theoretical Background

The fundamental mathematical concepts utilized by this simulation can be found in the following papers: [Link](https://drive.google.com/drive/folders/0By-Hy-bcfIQSNkNraGhRSUs0M0U?usp=sharing)

This simulation is based,and extends upon,the following papers: [Link](https://drive.google.com/drive/folders/0By-Hy-bcfIQSOUxsMUNGTHh5NnM?usp=sharing)

### Convex Cone method

This method is used when the expanding controller must force the state of the system through a specific facet of the desired operation area, in order to reach the chosen equilibrium point. In this case, the following procedure is followed :

* The convex hull of the chosen facet's vertices plus the initial point, is calculated. It is then converted in H-representation (system of linear inequalities).

* From the linear inequalities , we remove the one which will cause the remaining to represent a convex cone , starting from the initial point and facing towards the chosen facet and beyond.

* Via a theorem described in the papers (see above) , we calculate a suitable gain factor so that the equilibrium point (which resides within the convex cone) is reached in the least amount of time steps.


## How to run

Before running, unzip the 'tbxmanager' folder. The 'tbxmanager' folder contains the MPT3 library. The simulation can then be executed by running the 'SIMULATION.m' file via MATLAB. The rest of the folders contain the definitions of the functions used by the simulation .

## Features

* Default input data , for testing purposes.

* Choice of 3 methods for expanding controller, in the covert case.

* Option to show all candidate equilibrium points for expanding controller, in the covert case .

* Option to introduce uncertainty in the state position for the expanding controller, in the regular case.

## Screenshots
<img src="https://user-images.githubusercontent.com/20325266/30706674-cb454e08-9f01-11e7-98c9-21c9cfb597c6.jpg" width="49%"><img src ="https://user-images.githubusercontent.com/20325266/30706675-cc753234-9f01-11e7-919e-0a8e4f0d5d95.jpg" width="49%">

<img src="https://user-images.githubusercontent.com/20325266/30706668-c70d392c-9f01-11e7-9cb0-3469f1c71386.jpg" height="150%" width = "50%"><img src="https://user-images.githubusercontent.com/20325266/30706669-c8a43150-9f01-11e7-9ede-acd2ecadea73.jpg" width="49%">

<img src="https://user-images.githubusercontent.com/20325266/30706661-c2b1947c-9f01-11e7-8a49-a1a507733c51.jpg" width="49%"><img src="https://user-images.githubusercontent.com/20325266/30706667-c5510f50-9f01-11e7-8d5f-b0aa0ab33ac9.jpg" width="49%">


<img src="https://user-images.githubusercontent.com/20325266/30706673-c9f4d942-9f01-11e7-9c73-0901d665284b.jpg" width="49%"><img src="https://user-images.githubusercontent.com/20325266/30773967-258a62da-a084-11e7-9de1-9dd912a8cc1c.jpg" width="49%">


## Coming Soon

* Support for multi-dimensional systems

* File I/O support.

* Improvements on mathematical optimization.

* Improvements on convex cone method.

## Dependencies

This simulation uses the 'Multi-Parametric Toolbox 3 (MPT3)' library for the calculation and
plotting of the convex cone.

It will also be used in the future, for the solution of the linear and nonlinear programming problems
in the case of multi-dimensional systems.

MPT3 is licensed under GPL.

( http://people.ee.ethz.ch/~mpt/3/ )

## License

This simulation is distributed under Apache License Version 2.0

Copyright (C) 2017 Stylianos Tsiakalos
