function [delta_ff] = feedforward_steering(lat_mpc,vx,kappa_ref)
%FEEDFORWARD_STEERING computes a good "probable" initial condition for the
%steering position. Can also be used to cyclically compute a feedforward
%steering angle
lat= lat_mpc.sys_params;
K_v_ff            = ((lat.m*lat.l_r)/(2*lat.C_f*(lat.l_r+lat.l_f)))-((lat.m*lat.l_f)/(2*lat.C_r*(lat.l_r+lat.l_f)));
a_y_ff            =vx^2*kappa_ref;
delta_ff          = (lat.l_r+lat.l_f)*kappa_ref + K_v_ff*a_y_ff - 1*((lat.l_r*kappa_ref)-((lat.l_f*lat.m*vx^2*kappa_ref)/(2*lat.C_r*(lat.l_r+lat.l_f))));
end

