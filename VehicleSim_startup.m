clc; clear; close all;

%% Options

addpath("EKF/","MPC/","PLANT/","MRAC/","REFERENCE/","HELPERS/");
ekf_switch = false; % Do you want the parameter estimator on?
mrac_switch= true; % Do you want the MRAC on?

plots= true; % Do you want to plot the results?

%% Load Reference and Setup Simulation
simTs= 0.001; % Simulation Time Step
controlTs= 0.01; % Controller Time Step

ref=ReferenceGenerator(controlTs,false,3); 
% Ins-> Controller sampling time,plotting reference option, choice between 
% 1. oval track
% 2. figure 8 60 kmph
% 3. figure8 45 kmph 

Tsim= ref.t_ref(end); % Simulation time limit

t= (0:simTs:Tsim)'; % Simulation time span

%% Load Nonlinear Vehicle Dynamics as Plant
% Plant States Order -> X,Y,psi,xDot,yDot,psiDot,theta,thetaDot,phi,phiDot
% omega1,omega2,omega3,omega4,alpha1,alpha2,alpha3,alpha4
% Inputs order -> Tau_F_cmd, Tau_R_cmd, delta_cmd

plant=VEHICLE(setupVehicleParams,simTs,1); 
% Vehicle class initiates all the properties and methods for the plant
% Ins for constructor-> struct of vehicle parameters, simulation timestep,
% surface condition of sim (1->ashphalt, 2->snow, 3->rain, 4->ice

X_pl_Record= zeros(18,floor(t(end)/simTs));
X_pl_Record(:,1)=[ref.x_ref(1);ref.y_ref(1);ref.psi_ref(1);ref.v_ref(1);0; ...
              ref.v_ref(1)*ref.curvature_ref(1); 0; 0; 0; 0; ...
              ref.v_ref(1)/plant.Rw;ref.v_ref(1)/plant.Rw; ...
              ref.v_ref(1)/plant.Rw;ref.v_ref(1)/plant.Rw; ...
              0;0;0;0];

XDot_pl_Record = zeros(18,floor(t(end)/simTs));
U_Record= zeros(4,floor(t(end)/simTs));
err_vec= zeros(4,length(ref.x_ref));

%% Setup the MPCs
% MPC_CONTROLLER class uses osqp library to set up an MPC solver
lat_mpc= MPC_CONTROLLER(setupLatMpcParams(plant,controlTs));
lon_mpc= MPC_CONTROLLER(setupLonMpcParams(plant,controlTs));

%% Setup the EKF
if ekf_switch
    X_hat_Record= zeros(6,floor(t(end)/controlTs));
    X_hat_Record(:,1)  =[0;ref.v_ref(1)*ref.curvature_ref(1);0;0;lat_mpc.sys_params.C_f;lat_mpc.sys_params.C_r];
    P= 1e-6*eye(6);

    ekf= setupEKFparams(lat_mpc.sys_params,controlTs);
end

%% Setup the MRAC
if mrac_switch
 % Setup an object of MRAC controller and pass arguments to constructor
    mrac= MRAC_CONTROLLER(setupMRAC(lat_mpc.sys_params,X_pl_Record(4,1),controlTs));
    err_mrac_vec= zeros(4,floor(t(end)/controlTs));
    X_MRAC_bar_vec= zeros(4,floor(t(end)/controlTs));
    X_MRAC_vec= zeros(4,floor(t(end)/controlTs));
    gains_MRAC_vec= zeros(9,floor(t(end)/controlTs)); % Kdel,Ky,Ke
end

%% Run the simulation
main;


lat_error= calc_LateralDeviation(X_pl_Record(1,:),X_pl_Record(2,:),...
                                 ref.x_ref,ref.y_ref,X_pl_Record(3,:),...
                                 simTs,controlTs);

 rms_impr_perc = ((ref.lat_error.rms-lat_error.rms)/lat_error.rms)*100
 max_impr_perc = ((ref.lat_error.max-lat_error.max)/lat_error.max)*100

%% Plots
if plots
   plotter;
end

rmpath("EKF/","MPC/","PLANT/","MRAC/","REFERENCE/","HELPERS/");