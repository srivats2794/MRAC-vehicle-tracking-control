function vehicle_params=setupVehicleParams()
vehicle_params.sr= 1/20; % Steering ratio

vehicle_params.m = 2100;   % mass of car

vehicle_params.Ixx = 765;

vehicle_params.Iyy = 3477; 

vehicle_params.Izz = 3900;% Moment of inertia
vehicle_params.l_f = 1.3; % Front axle to center of gravity
vehicle_params.l_r = 1.5; % Rear axle to center of gravity
vehicle_params.l= vehicle_params.l_f+vehicle_params.l_r; 
vehicle_params.w = 0.8;  
vehicle_params.Rw = 0.3; 
vehicle_params.Iw = 4.0; 
vehicle_params.sig = 0.3; 
vehicle_params.h = 0.5;   
% 
vehicle_params.K_theta = 363540; 
vehicle_params.D_theta = 30960; 
% 
vehicle_params.K_phif = 89000; 
vehicle_params.K_phir = 89000; 
vehicle_params.K_phi = vehicle_params.K_phif+vehicle_params.K_phir; 
vehicle_params.D_phif = 8000; 
vehicle_params.D_phir = 8000; 
vehicle_params.D_phi = vehicle_params.D_phif+vehicle_params.D_phir; 


vehicle_params.scale = 1e3;
vehicle_params.epsilon = 1e-6;
end
