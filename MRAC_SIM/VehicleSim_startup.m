function VehicleSim_startup(scenario,plots)

clc; clear; close all;

if nargin<2
    plots=1; % Do you want the plots or no?
end
if nargin<1
    scenario=3; % MANIPULATE THE PARAMETER PERTURBATION PERCENTAGE
end

addpath("MPC/","PLANT/","MRAC/","REFERENCE/","HELPERS/");

%% Load Reference and Setup Simulation
simTs = 0.001; % Simulation Time Step
controlTs = 0.01; % Controller Time Step

ref=ReferenceGenerator(controlTs,false,scenario); 
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

plant=VEHICLE(setupVehicleParams(),simTs,1); 
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

%% Setup the Lon MPC
% MPC_CONTROLLER class uses osqp library to set up an MPC solver
lon_mpc= MPC_CONTROLLER(setupLonMpcParams(plant,controlTs));

%% Setup the MRAC
% Setup an object of MRAC controller and pass arguments to constructor

% Vehicle params for the controller. C_f and C_r are initialized with rough
% vals
temp= setupLatMpcParams(plant,controlTs);  
lat_control_params= temp.sys_params;
clearvars temp

mrac= MRAC_CONTROLLER(setupMRAC(lat_control_params,X_pl_Record(4,1),controlTs));
err_mrac_vec= zeros(4,floor(t(end)/controlTs));
X_MRAC_bar_vec= zeros(4,floor(t(end)/controlTs));
X_MRAC_vec= zeros(4,floor(t(end)/controlTs));
gains_MRAC_vec= zeros(9,floor(t(end)/controlTs)); % Kdel , Ky, Ke

%% Run the simulation
main;

lat_error= calc_LateralDeviation(X_pl_Record(1,:),X_pl_Record(2,:),...
    ref.x_ref,ref.y_ref,X_pl_Record(3,:),...
    simTs,controlTs);

RMS_Ey = ((ref.lat_error.rms-lat_error.rms)/lat_error.rms)*100;
Max_Ey = ((ref.lat_error.max-lat_error.max)/lat_error.max)*100;

disp(['RMS Lateral Deviation Improvement From EKF-MPC = ' num2str(RMS_Ey)]);
disp(['Max Lateral Deviation Improvement From EKF-MPC = ' num2str(Max_Ey)]);
%% Plots
if plots
   plotter;
end

rmpath("MPC/","PLANT/","MRAC/","REFERENCE/","HELPERS/");