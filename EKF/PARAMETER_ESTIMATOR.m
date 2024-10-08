classdef PARAMETER_ESTIMATOR
    % Class to set up an EKF based cornering stiffness estimator for vehicles
   
    properties
       Ts
       MeasUC % Measurement Uncertainty
       ProcUC % Process Uncertainty
       m,l_f,l_r,C_f,C_r,I_z
       C_f_bound_perc % Max Cornering stiffness front for constrained estimation
       C_r_bound_perc % Max Cornering stiffness rear for constrained estimation
    end
    
    methods
        function obj = PARAMETER_ESTIMATOR(vehicle,Ts,MeasUC,ProcUC,C_f_bounds,C_r_bounds)
            obj.Ts= Ts;
            obj.MeasUC= MeasUC;
            obj.ProcUC= ProcUC;
            obj.m= vehicle.m;
            obj.C_f= vehicle.C_f;
            obj.C_r= vehicle.C_r;
            obj.I_z= vehicle.I_z;
            obj.l_f= vehicle.l_f;
            obj.l_r= vehicle.l_r;
            obj.C_f_bound_perc=C_f_bounds;            
            obj.C_r_bound_perc= C_r_bounds;         
        end
        
        function [psiDot_est,beta_est,C_f_est,C_r_est,P,X_hat] = estimate(obj,X_hat,P,psiDot,Ay,Vx,delta)
            % Parameter estimation loop
            
            % Covariance of process and measurement noises for EKF algorithm
            
            Rkf         =diag(obj.MeasUC.^2);
            Qkf         =diag(obj.ProcUC.^2);

            %-- measurement matrix
            Hk         =[0 1 0 0 0 0;0 Vx Vx 0 0 0];

            %-- Covariance matrix
            Bwk      =[0                             0 0;...
                0                             0 0;...
                2*X_hat(5)/(obj.m*Vx)          0 0;...
                (2*X_hat(5)*obj.l_f)/obj.I_z     0 0;...
                0                             1 0;...
                0                             0 1];

            Qdk      =obj.Ts*Bwk*Qkf*Bwk';


            %-- measurement output: yaw rate and lateral acceleration
            ym         =[psiDot; Ay];
            
            %-- Linearized system with nominal parameters for filter

            A31       =-(X_hat(5)+X_hat(6))/(obj.m*Vx);
            A32       =-(((X_hat(5)*obj.l_f-X_hat(6)*obj.l_r)/(obj.m*Vx^2))+1);
            A35       =2*((delta/(obj.m*Vx)) - (X_hat(1)/(obj.m*Vx)) - ((obj.l_f*X_hat(2))/(obj.m*Vx^2)));
            A36       =2*(((obj.l_r*X_hat(2))/(obj.m*Vx^2)) -(X_hat(1)/(obj.m*Vx)));

            A41       =-(X_hat(5)*obj.l_f - X_hat(6)*obj.l_r)/(obj.I_z);
            A42       =-(X_hat(5)*obj.l_f^2 + X_hat(6)*obj.l_r^2)/(obj.I_z*Vx);
            A45       =2*(+((delta*obj.l_f)/obj.I_z) - ((obj.l_f^2*X_hat(2))/(obj.I_z*Vx)) - ((obj.l_f*X_hat(1))/obj.I_z));
            A46       =2*(+((obj.l_r*X_hat(1))/obj.I_z) -((obj.l_r^2*X_hat(2))/(obj.I_z*Vx)));

            Ak        =[ 1,    0,  obj.Ts,     0,         0,   0;
                         0,    1,       0,      obj.Ts,   0,   0;
                        A31, A32,      0,         0,       A35, A36;
                        A41, A42,      0,         0,       A45, A46;
                         0,    0,       0,         0,         1,   0;
                         0,    0,       0,         0,         0,   1];

            %*** (I) priori estimation

            %--estimated nonlinear system with nominal parameters

            % X_hat = [beta_hat   psiDot_hat    betaDot_hat   psiDDot_hat   C_f_hat      C_r_hat] \in R^6
            
            X_hat        =[X_hat(1) + X_hat(3)*obj.Ts;...
                          X_hat(2) + X_hat(4)*obj.Ts;...
                         -((2*X_hat(5)+2*X_hat(6))/(obj.m*Vx))*X_hat(1) - (((2*X_hat(5)*obj.l_f-2*X_hat(6)*obj.l_r)/(obj.m*Vx^2))+1)*X_hat(2) +  (2*X_hat(5)*delta)/(obj.m*Vx);...
                         -((2*X_hat(5)*obj.l_f-2*X_hat(6)*obj.l_r)/obj.I_z)*X_hat(1) - ((2*X_hat(5)*obj.l_f^2+2*X_hat(6)*obj.l_r^2)/(obj.I_z*Vx))*X_hat(2) +  (2*X_hat(5)*delta*obj.l_f)/obj.I_z;...
                          X_hat(5);...
                          X_hat(6)];

            %-- discrete-time covariance
            P         =Ak*P*Ak'+Qdk;

            %*** (II) posteriori estimation

            % --discrete kalman gain
            K         =P*Hk'/(Hk*P*Hk'+Rkf);

            % --discrete estimated state update
            X_hat      =X_hat+K*(ym-Hk*X_hat);

            % discrete covariance
            P        =(eye(size(Qdk))-K*Hk)*P;

            %--Project state estimate onto inequality constraints for bounding the CSs

            % --max/min values allowed
            C_f_max         =obj.C_f+obj.C_f*obj.C_f_bound_perc;
            C_f_min         =obj.C_f-obj.C_f*obj.C_f_bound_perc;
            C_r_max         =obj.C_r+obj.C_r*obj.C_r_bound_perc;
            C_r_min         =obj.C_r-obj.C_r*obj.C_r_bound_perc;

            %--upper/lower bounds
            lb              = [C_f_min; C_r_min];
            ub              = [C_f_max; C_r_max];
            Mx              = [0 0 0 0 1 0; 
                               0 0 0 0 0 1;
                               0 0 0 0 -1 0; 
                               0 0 0 0 0 -1;];
            dx              = [ub; -lb];

            %-- projection operator to be implemented
            % X_hat             = obj.proj(X_hat, Mx, dx);

            %-- post-operation for boundedness
            X_hat(5)          = min(X_hat(5),C_f_max);
            X_hat(5)          = max(X_hat(5),C_f_min);
            X_hat(6)          = min(X_hat(6),C_r_max);
            X_hat(6)          = max(X_hat(6),C_r_min);

            % X_hat = [beta_hat   psiDot_hat    betaDot_hat   psiDDot_hat   C_f_hat      C_r_hat] \in R^6
            beta_est = X_hat(1);
            psiDot_est = X_hat(2);
            C_f_est = X_hat(5);
            C_r_est = X_hat(6);
        end

        function [X] = proj(X,Mx,d)
            % Project X onto the region defined by Mx <= d using the projected gradient method

            % To be formulated
            max_iter = 1e3;
            step_size = 1;
            tol = 1e-16;
        end
    end
end
