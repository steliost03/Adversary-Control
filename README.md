# Adversary Control

## Overview

Simulation of adversary control in a linear, 2-dimensional discrete system in state-space representation (soon to be expanded to higher dimensional systems) .

We assume that the system operates under linear constraints regarding both the input of the controllers and regarding its state. Depending on the type of scenario (see below) a specific area is defined, called the desired operation area, within which the system wants to operate.

The system is controlled by two opposing parties : The contractive and the expanding controllers.
The objective of the contractive controller is to keep the system operating in a stable equilibrium point within the desired operation area , while the objective of the expanding controller is to disrupt this stability and force it to operate outside
of it (while respecting input constraints).



## Theoretical Background

The fundamental mathematical concepts utilized by this simulation can be found in the following papers: [Link](https://drive.google.com/drive/folders/0By-Hy-bcfIQSNkNraGhRSUs0M0U?usp=sharing)

This simulation is based,and extends upon,the following papers: [Link](https://drive.google.com/drive/folders/0By-Hy-bcfIQSOUxsMUNGTHh5NnM?usp=sharing)


## How to run

Before running, unzip the 'tbxmanager' folder. The 'tbxmanager' folder contains the MPT3 library. The simulation can then be executed by running the 'SIMULATION.m' file via MATLAB. The rest of the folders contain the definitions of the functions used by the simulation .

## Features

## Screenshots
<img src="https://user-images.githubusercontent.com/20325266/30706674-cb454e08-9f01-11e7-98c9-21c9cfb597c6.jpg" width="49%"><img src ="https://user-images.githubusercontent.com/20325266/30706675-cc753234-9f01-11e7-919e-0a8e4f0d5d95.jpg" width="49%">

<img src="https://user-images.githubusercontent.com/20325266/30706668-c70d392c-9f01-11e7-9cb0-3469f1c71386.jpg" height="150%" width = "50%"><img src="https://user-images.githubusercontent.com/20325266/30706669-c8a43150-9f01-11e7-9ede-acd2ecadea73.jpg" width="49%">

<img src="https://user-images.githubusercontent.com/20325266/30706661-c2b1947c-9f01-11e7-8a49-a1a507733c51.jpg" width="49%"><img src="https://user-images.githubusercontent.com/20325266/30706667-c5510f50-9f01-11e7-8d5f-b0aa0ab33ac9.jpg" width="49%">



<img src="https://user-images.githubusercontent.com/20325266/30706673-c9f4d942-9f01-11e7-9c73-0901d665284b.jpg" width="49%">




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

Copyright (C) 2017 Stelios Tsiakalos
