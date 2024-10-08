load("steering_ref_dlc.mat"); steer_input=ref_steering_angle.Data(:,1);
input_time= ref_steering_angle.Time;



steer_input= deg2rad(plant.sr*makima(input_time,steer_input,t));

clearvars input_time ref_steering_angle