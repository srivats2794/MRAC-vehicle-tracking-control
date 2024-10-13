clc;clear all; close all;
scenario=2; % 1 for Closed Course. 2 for Figure8
plots=0;

%% Vanilla
cd MPC_SIM/
    mpc_lat_error= VehicleSim_startup(scenario,plots);
cd ..

%% EKF-MPC
cd EKF_MPC_SIM/
    ekf_mpc_lat_error= VehicleSim_startup(scenario,plots);
cd ..

%% MRAC
cd MRAC_SIM/
    mrac_error= VehicleSim_startup(scenario,plots);
cd ..

cd ROBUST_MRAC_SIM/
    robust_mrac_error= VehicleSim_startup(scenario,plots);
cd ..

clc;
RMS_Ey = ((mpc_lat_error.rms-ekf_mpc_lat_error.rms)/ekf_mpc_lat_error.rms)*100;
Max_Ey = ((mpc_lat_error.max-ekf_mpc_lat_error.max)/ekf_mpc_lat_error.max)*100;

disp(['RMS Lateral Deviation Improvement by EKF-MPC = ' num2str(RMS_Ey)]);
disp(['Max Lateral Deviation Improvement by EKF-MPC = ' num2str(Max_Ey)]);

RMS_Ey = ((mpc_lat_error.rms-mrac_error.rms)/mrac_error.rms)*100;
Max_Ey = ((mpc_lat_error.max-mrac_error.max)/mrac_error.max)*100;

disp(['RMS Lateral Deviation Improvement by MRAC = ' num2str(RMS_Ey)]);
disp(['Max Lateral Deviation Improvement by MRAC = ' num2str(Max_Ey)]);

RMS_Ey = ((mpc_lat_error.rms-robust_mrac_error.rms)/robust_mrac_error.rms)*100;
Max_Ey = ((mpc_lat_error.max-robust_mrac_error.max)/robust_mrac_error.max)*100;

disp(['RMS Lateral Deviation Improvement by Robust MRAC = ' num2str(RMS_Ey)]);
disp(['Max Lateral Deviation Improvement by Robust MRAC = ' num2str(Max_Ey)]);