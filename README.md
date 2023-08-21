# The Cost and Risk-reducing Quadcopter System
 Simulation environment for the quadcopter testbed named CRQS.
## Files
- **compile.m**: Runs all other necessary scripts for the initial conditions selected within this script.
- **constants.m**: Defines all vehicle parameters.
- **memory_prealloc.m**: Defines variables that are used within the numerical integration to reduce computational expense.
- **sim_loop.m**: Script run from **compile.m** that numerically integrates the continuous-time system with discrete-time inputs.
- **postprocessing.m**: Takes output data from simulation and prepares it for **plotscript.m**.
- **plotscript.m**: Plots the positions, velocities, attitudes, angular velocities, tip deflection, energy check, DCM check, control effort, and estimation error.

## Folders
- **files_for_animation**: Contains scripts to create a video file of the CRQS moving in 3D space.
- **files_for_control**: Contains scripts to synthesize different control and estimation algorithms.
- **files_for_dynamics**: Contains scripts for computing inverted pendulum parameters and the ODEs file that is numerically integrated in **compile.m**/**sim_loop.m**.
- **files_for_support**: Contains functions and scripts that support the primary MATLAB files.
- **files_for_trajectory**: Contains files to compute a desired guidance trajecotry.
- **results**: Contains folders of various run cases. Each of these subfolders contains the data from the tests indicated by the folder name. Allows one to simply run the post-processing and plot scripts to replicate figures instead of running the simulation again.
# CRQS-Simulation
