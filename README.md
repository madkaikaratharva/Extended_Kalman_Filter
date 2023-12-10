# Extended Kalman Filter for a Quadrotor

## Overview :

The aim of this lab assignment was to implement an Extended Kalman Filter (EKF) for a Quadrotor to estimate its NED Positions and NED velocities. Motor signals and GPS data was provided which was used for position and velocity estimates. The GPS co-ordinates (Geodetic) needed to be converted to ECEF co-ordinates and eventually to NED co-ordinates to be used by the EKF for state estimation. This EKF implementation further improved state estimations by estimating acceleration biases as well, enhancing predictive accuracy.

A detailed explainantion of the Prediction Model, Implemented code, and the Data has been provided in the report.


## Usage :

- Kindly run the **EKF_Quadrotor.m** script to load and run the EKF on the provided data. The script also produces detailed plots which offer insights into the input parameters, state estimations and acceleration biases.

- The MATLAB file **data.mat** contains the GPS data, Motor Signal Plots and the IMU readings which are used by the EKF.


