# Cooperative collision avoidance using horizontal maneouvres only.

A final year project completed by Adrian Stark [25855670] for the Mechatronic Project 488 module.\
This repository serves as a project file for examination purposes.

## Folders
- Videos: contains videos from the testing process to demonstrate the system in action.
- Simulation: all files relating to the developed simulation of the demonstrated system.
- Demonstration: all files needed to control the veehicles and follow their paths. Also includes the code for the vehicles.

## Simulation
The simulation can be run if the user has MATLAB and Simulink installed on their PC.
1. Download or copy the repository onto your PC.
2. Navigate to the `Simulation` folder and open `route_planner.m` and `run_simulation.m`.
3. Adjust the start and end locations in `route_planner.m`, and run it. It will output `vehicle_paths.csv` and give a plot of the paths that the route planner solved.
4. Run `run_simulation.m`. It may need to recreate the simulation models if it does not use the cached ones.
5. It will output a dashboard image of the simulated system.
6. Change the paths and variables and have fun playing with the vehicles.
