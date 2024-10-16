%******* This script plots the results generated by the controller

%% Time and variable extraction
t                          =out.time;
x                          =out.States;
size_x                     =size(x);
Estimates                  =out.Estimates;
sleep                      =out.sleep;
yaw_rate                   =out.yaw_rate;
steering_ang_feedback      =out.steering_ang_feedback;
N_estimates_lat            =out.N_estimates_lat;
N_estimates_long           =out.N_estimates_long;
feed                       =out.feedbacks;
u_long_feed                =out.u_long_feed;
roll_over_index            =out.roll_over_index;
hR                         =out.hR;
T_e_feedback               =out.T_e_feedback;
T_b_feedback               =out.T_b_feedback;
T_b_feedback_direct        =out.T_b_feedback_direct.Data;

for i=1:size_x(3)
    e_y(i,:)             =x(1,1,i);
    e_yDot(i,:)          =x(2,1,i);
    e_phi(i,:)           =x(3,1,i);
    e_phiDot(i,:)        =x(4,1,i);
    steer_measur(i,:)    =x(5,1,i);
    u_Rec(i,:)           =out.u_Rec(:,:,i);
    ctrl_Rec(i,:)        =out.ctrl_Rec(:,:,i);
    ff_Rec(i,:)          =out.ff_Rec(:,:,i);
    steering_ang(i,:)    =out.steering_ang(:,:,i);
    u_long_Rec(i,:)      =out.u_long_Rec(:,:,i);
    ctrl_long_Rec(i,:)   =out.ctrl_long_Rec(:,:,i);
    ff_long_Rec(i,:)     =out.ff_long_Rec(:,:,i);
    T_e_command(:,i)     =out.T_e_command(:,:,i);
    T_b_command(:,i)     =out.T_b_command(:,:,i);
end

%% reference variables
V_x_ref                 =out.V_x_ref_out;
X_ref                   =out.X_ref_out;
Y_ref                   =out.Y_ref_out;
PHI_ref                 =out.phi_ref_out;
kappa_ref               =out.kappa_ref_out;

%% actual variable calculated
phi_real_calc                =e_phi+PHI_ref;
for i=1:size_x(3)
    X_calc(i)                =X_ref(i)-e_y(i)*sin(phi_real_calc(i));
    Y_calc(i)                =Y_ref(i)+e_y(i)*cos(phi_real_calc(i));
end

%% actual variable measured
V_actual              =feed(:,1);
V_y_actual            =feed(:,2);
phi_real_m            =feed(:,3);
yaw_rate_actual       =feed(:,4);
X_m                   =feed(:,5);
Y_m                   =feed(:,6);

%% velocity error
e_V                   =V_actual-V_x_ref;

%% EKF estimator
beta_hat              =Estimates(:,1);
yaw_rate_hat          =Estimates(:,2);
Cf_hat                =Estimates(:,5);
Cr_hat                =Estimates(:,6);

%% lateral acceleration
ay                            =V_actual.^2.*kappa_ref;
ay_orginal                    =(V_x_ref_original.^2).*kappa_ref_original;
ay_shaped                     =(V_x_ref_shaper.^2).*kappa_ref_original;

%% error states
figure(1),
subplot(221),
plot(t,e_y,'LineWidth',2)
ylabel('$$e_{y}$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
grid
subplot(222),
plot(t,e_yDot,'LineWidth',2)
ylabel('$$e_{\dot{y}}$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
grid
subplot(223),
plot(t,e_phi,'LineWidth',2)
ylabel('$$e_{\phi}$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
grid
subplot(224),
plot(t,e_phiDot,'LineWidth',2)
ylabel('$$e_{\dot{\phi}}$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
grid

%% X-Y plot
figure(2)
plot(X_calc,Y_calc,'b','LineWidth',2)
ylabel('Y (m)','interpreter','latex')
xlabel({'X (m)'},'interpreter','latex')
hold on
plot(X_m,Y_m,'k:','LineWidth',2)
hold on
plot(X_ref,Y_ref,'r--','LineWidth',2)
legend({'Calculated','Actual','Reference'},'Interpreter','latex')
grid

%% steering (total, feedback, feedforward)
figure(3)
plot(t,u_Rec,'b','LineWidth',2)
ylabel('$$\delta$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
hold on
plot(t,ctrl_Rec,'r--','LineWidth',2)
hold on
plot(t,ff_Rec,'k:','LineWidth',2)
legend({'total','feedback','feedforward'},'Interpreter','latex')
grid

%% steering (command, feedback)
figure(4),
plot(t,steering_ang,'b','LineWidth',2)
ylabel('$$\delta$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
hold on
plot(t,steering_ang_feedback,'r--','LineWidth',2)
legend({'Command','Feedback'},'Interpreter','latex')
grid

%% velocity
figure(5)
plot(t,V_actual,'b','LineWidth',2)
ylabel('$$V$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
hold on
plot(t,V_x_ref,'r--','LineWidth',2)
hold on
legend({'actual','reference'},'Interpreter','latex')
grid

%% ax (total, feedback, feedforward)
figure(6)
plot(t,u_long_Rec,'b','LineWidth',2)
ylabel('$$a_x$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
hold on
plot(t,ctrl_long_Rec,'r--','LineWidth',2)
hold on
plot(t,ff_long_Rec,'k:','LineWidth',2)
legend({'total','feedback','feedforward'},'Interpreter','latex')
grid

%% ax(command, feedback)
figure(7),
plot(t,u_long_Rec,'b','LineWidth',2)
ylabel('$$a_x$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
hold on
plot(t,out.u_long_feed,'r--','LineWidth',2)
legend({'Command','State'},'Interpreter','latex')
grid

%% sleep angle and yaw rate estimates
figure(8),
subplot(211),
plot(t,beta_hat,'b','LineWidth',2)
ylabel('$$\beta$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
hold on 
plot(t,sleep,'r--','LineWidth',2)
legend({'Estimate','Real'},'Interpreter','latex')
grid
subplot(212),
plot(t,yaw_rate_hat,'b','LineWidth',2)
ylabel('$$\dot{\theta}$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
hold on 
plot(t,yaw_rate,'r--','LineWidth',2)
grid

%% CSs
figure(9),
subplot(211),
plot(t,Cf_hat,'b','LineWidth',2)
ylabel('$$\hat{C}_f$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
hold on 
grid
subplot(212),
plot(t,Cr_hat,'b','LineWidth',2)
hold on 
ylabel('$$\hat{C}_r$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
grid

%% estimates of N
figure(10),
subplot(211),
plot(t,N_estimates_lat,'b','LineWidth',2)
ylabel('$$\hat{N}_{lat}$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
hold on
grid
subplot(212),
plot(t,N_estimates_long,'b','LineWidth',2)
ylabel('$$\hat{N}_{lon}$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
grid

%% rollover index, lateral accelration, height of vehicle c.g. from roll center
figure(11),
subplot(311),
plot(t,roll_over_index,'b','LineWidth',2)
ylabel('$$R_{roll}$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
grid
subplot(312),
plot(t,ay,'b','LineWidth',2)
ylabel('$$a_y$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
grid
subplot(313),
plot(t,hR,'b','LineWidth',2)
ylabel('$$h_R$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
title('height of vehicle c.g. from roll center')
grid

%% velocity and lateral acceleration after/before shaping
figure(12),
subplot(211),
plot(t_original,V_x_ref_original,'b','LineWidth',2)
hold on
plot(t_original,V_x_ref_shaper,'r--','LineWidth',2)
ylabel('$$V_x$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
legend({'Original','Shaper'},'Interpreter','latex')
grid
subplot(212),
plot(t_original,ay_orginal,'b','LineWidth',2)
hold on
plot(t_original,ay_shaped,'r--','LineWidth',2)
ylabel('$$a_y$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
grid

%% engine and brake torques (command, feedback)
figure(13),
subplot(211),
plot(t,T_e_command,'b','LineWidth',2)
hold on
plot(t,T_e_feedback,'r--','LineWidth',2)
ylabel('$$T_e$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
legend({'command','feedback direct'},'Interpreter','latex')
grid
subplot(212),
plot(t,T_b_command,'b','LineWidth',2)
hold on
plot(t,T_b_feedback_direct,'r--','LineWidth',2)
hold on
plot(t,T_b_feedback,'k:','LineWidth',2)
ylabel('$$T_b$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
legend({'command','feedback direct','feedback LUT'},'Interpreter','latex')
grid

%% commanded engine and brake torques
figure(14),
plot(t,T_e_command,'b','LineWidth',2)
hold on
plot(t,T_b_command,'r--','LineWidth',2)
ylabel('$$T$$','interpreter','latex')
xlabel({'Time (sec)'},'interpreter','latex')
legend({'$$T_e$$','$$T_b$$'},'Interpreter','latex')
title('Command torques')
grid

%% numerical results
RMS_y=sqrt(sum(e_y.^2)/length(e_y));
RMS_yDot=sqrt(sum(e_yDot.^2)/length(e_yDot));
RMS_phi=sqrt(sum(e_phi.^2)/length(e_phi));
RMS_phiDot=sqrt(sum(e_y.^2)/length(e_phiDot));
RMS_del=sqrt(sum(u_Rec.^2)/length(u_Rec));
RMS_vel=sqrt(sum(e_V.^2)/length(e_V));
RMS_accel=sqrt(sum(u_long_Rec.^2)/length(u_long_Rec));

disp('------------**Lat Results**---------------')
disp(['RMSE_y                 = ' num2str(RMS_y)])
disp(['RMSE_yDot              = ' num2str(RMS_yDot)])
disp(['RMSE_phi               = ' num2str(RMS_phi)])
disp(['RMSE_phiDot            = ' num2str(RMS_phiDot)])
disp(['RMS_delta              = ' num2str(RMS_del)])
disp('------------**Long Results**---------------')
disp(['RMSE_vel               = ' num2str(RMS_vel)])
disp(['RMS_accel              = ' num2str(RMS_accel)])
disp('------------**Rollover**---------------')
disp(['RollOverIndex          = ' num2str(max(out.roll_over_index))])

% EKF.Vx= V_actual;
% EKF.t= t;
% EKF.Vy= V_y_actual;
% EKF.psi= phi_real_m;
% EKF.psiDot= yaw_rate_actual;
% EKF.X= X_m;
% EKF.Y= Y_m;
% EKF.C_f= out.Cf_real_all;
% EKF.C_r= out.Cr_real_all;
% EKF.delta_cmd= u_Rec;
% EKF.Ax_cmd = u_long_Rec;
% EKF.C_f_init= C_f_0;
% EKF.C_r_init= C_r_0;
% EKF.X_ref= out.X_ref_out;
% EKF.Y_ref= out.Y_ref_out;
% EKF.psi_ref= out.phi_ref_out;
% EKF.Vx_ref= out.V_x_ref_out;
% EKF.X_ref= out.X_ref_out;
% EKF.Y_ref= out.Y_ref_out;
% EKF.psi_ref= out.phi_ref_out;
% EKF.kappa_ref= out.kappa_ref_out;
% EKF.C_f_est= Cf_hat;
% EKF.C_r_est= Cr_hat;