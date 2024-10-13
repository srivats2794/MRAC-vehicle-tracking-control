classdef VEHICLE < handle
    % A class that sets up the nonlinear vehicle dynamics plant with
    % parameters and the neccessary methods needed to simulate it.
    properties
        g=9.81,m,Ixx,Iyy,Izz,l_f,l_r,l,w,Rw,Re,Iw,sig,h,K_theta, D_theta
        K_phif, K_phir, K_phi, D_phif,D_phir, D_phi
        epsilon, scale
        C_f, C_r
        sr %(Steering ratio)
        Ts
        tire_params
    end
    
    methods
        function obj= VEHICLE(vehicle_params,simSampleTime,surface)
            obj.sr = vehicle_params.sr;
            obj.m =vehicle_params.m ;

            obj.Ixx =vehicle_params.Ixx ;

            obj.Iyy = vehicle_params.Iyy;

            obj.Izz =vehicle_params.Izz;
            obj.l_f =vehicle_params.l_f;
            obj.l_r =vehicle_params.l_r ;
            obj.l=vehicle_params.l;
            obj.w = vehicle_params.w ;
            obj.Rw = vehicle_params.Rw;
            obj.Iw =vehicle_params.Iw;
            obj.sig = vehicle_params.sig;
            obj.h = vehicle_params.h ;
            
            obj.K_theta = vehicle_params.K_theta ;
            obj.D_theta =  vehicle_params.D_theta ;
            
            obj.K_phif = vehicle_params.K_phif;
            obj.K_phir = vehicle_params.K_phir;
            obj.K_phi =  vehicle_params.K_phi;
            obj.D_phif = vehicle_params.D_phif ;
            obj.D_phir = vehicle_params.D_phir;
            obj.D_phi =vehicle_params.D_phi;
          
            obj.scale = vehicle_params.scale ;
            obj.epsilon = vehicle_params.epsilon ;
            obj.Ts= simSampleTime;
            
            pacejkainit(obj,surface);

            Fzf= obj.m*obj.g*(obj.l_r/obj.l);
            Fzr= obj.m*obj.g*(obj.l_f/obj.l);

            Df= obj.tire_params(2)*Fzf;
            Dr= obj.tire_params(10)*Fzr;

            obj.C_f= 0.5*obj.tire_params(4)*obj.tire_params(6)*Df; % 0.5 is for per tire
            obj.C_r= 0.5*obj.tire_params(12)*obj.tire_params(14)*Dr;
        end
        function pacejkainit(this,surface)
            
                 if surface==1
                    % Ashpalt Front
                    this.tire_params(1)= 1.20 ; %mu_x_f
                    this.tire_params(2)= 0.935; %mu_y_f
                    this.tire_params(3)= 11.7; %B_x_f
                    this.tire_params(4)= 8.86; % B_y_f
                    this.tire_params(5)= 1.69; % C_x_f
                    this.tire_params(6)= 1.19; % C_y_f
                    this.tire_params(7)= 0.377; % E_x_f
                    this.tire_params(8)= -1.21; % E_y_f

                    % Ashpalt rear
                    this.tire_params(9)= 1.20;%mu_x_r
                    this.tire_params(10)= 0.961; %mu_y_r
                    this.tire_params(11)= 11.1; %B_x_r
                    this.tire_params(12)= 9.30; %B_y_r
                    this.tire_params(13)= 1.69; %C_x_r
                    this.tire_params(14)= 1.19; % C_y_r
                    this.tire_params(15)= 0.362; % E_x_r
                    this.tire_params(16)= -1.11; % E_y_r                
                 elseif surface==2
                    % Snow front
                    this.tire_params(1)= 0.407;
                    this.tire_params(2)= 0.383;
                    this.tire_params(3) = 10.2; 
                    this.tire_params(4)= 19.1;
                    this.tire_params(5)= 1.96; 
                    this.tire_params(6)= 0.550; 
                    this.tire_params(7)= 0.651; 
                    this.tire_params(8)= -2.1;

                    % Snow rear
                    this.tire_params(9) = 0.409;
                    this.tire_params(10)= 0.394;
                    this.tire_params(11)= 9.71;
                    this.tire_params(12)= 20.0;
                    this.tire_params(13)= 1.96;
                    this.tire_params(14)= 0.550; 
                    this.tire_params(15) = 0.624;
                    this.tire_params(16)= -1.93; 
                 
                 elseif surface==3
                    % Wet ashphalt front
                    this.tire_params(1)= 1.06;
                    this.tire_params(2)= 0.885;
                    this.tire_params(3) = 12;
                    this.tire_params(4)= 10.7;
                    this.tire_params(5)= 1.80;
                    this.tire_params(6)= 1.07;
                    this.tire_params(7)= 0.313;
                    this.tire_params(8)= -2.14;
                    
                    % Wet ashphalt rear
                    this.tire_params(9) = 1.07;
                    this.tire_params(10)= 0.911;
                    this.tire_params(11)= 11.5;
                    this.tire_params(12)= 11.3;
                    this.tire_params(13)= 1.80;
                    this.tire_params(14)= 1.07;
                    this.tire_params(15) = 0.300;
                    this.tire_params(16)= -1.97;
                 
                 else
                    % Smooth ice front
                    this.tire_params(1)= 0.172;
                    this.tire_params(2)= 0.162;
                    this.tire_params(3)= 31.1;
                   this.tire_params(4)= 28.4;
                    this.tire_params(5)= 1.77;
                    this.tire_params(6)= 1.48;
                    this.tire_params(7)= 0.710;
                    this.tire_params(8)= -1.18;
                    
                    % Smooth ice rear
                   this.tire_params(9) = 0.173;
                   this.tire_params(10)= 0.167;
                    this.tire_params(11)= 29.5;
                    this.tire_params(12)= 30;
                    this.tire_params(13)= 1.77;
                    this.tire_params(14)= 1.48;
                    this.tire_params(15) = 0.681;
                    this.tire_params(16)= -1.08;
                 end
        end
        function [statesDot,forces]= VehicleDynamics(this,states,inputs)

            x=states;       
            u=inputs;

            psi= x(3);
            Vx= x(4); Vy= x(5); psidot= x(6);
            theta= x(7); thetadot= x(8);
            phi= x(9); phidot= x(10);
            omega1= x(11); omega2= x(12);
            omega3= x(13); omega4= x(14);
            alpha1= x(15); alpha2= x(16);
            alpha3= x(17); alpha4= x(18);
            Tf= u(1); Tr= u(2); delta= u(3);

            %% Solving Fz (4 eqns 4 unknowns)
            tau_theta= (this.K_theta*theta)+(this.D_theta*thetadot);
            tau_phi= (this.K_phi*phi)+(this.D_phi*phidot);
            tau_phif= (this.K_phif*phi)+(this.D_phif*phidot);
            tau_phir= (this.K_phir*phi)+(this.D_phir*phidot);

            Fzf = (this.l_r*this.m*this.g + tau_theta)/(this.l);
            Fzr = this.m*this.g - Fzf;

            Fz1 = 1/2*(Fzf - 1/this.w*(tau_phif));
            Fz2 = Fzf - Fz1;
            Fz3 = 1/2*(Fzr - 1/this.w*(tau_phir));
            Fz4 = Fzr - Fz3;
            %% Vx and Vy at each wheel
            rot1= [cos(delta) -sin(delta);
                sin(delta) cos(delta)];
            Vx1_ur= Vx-(this.w*psidot); Vx2_ur= Vx+(this.w*psidot);
            Vx3_ur= Vx1_ur; Vx4_ur= Vx2_ur;
            Vy1_ur= Vy+(this.l_f*psidot); Vy3_ur= Vy-(this.l_r*psidot);
            Vy2_ur= Vy1_ur; Vy4_ur= Vy3_ur;
            Vt1 = rot1*[Vy1_ur; Vx1_ur];
            Vy1= Vt1(1); Vx1= Vt1(2);
            Vt2 = rot1*[Vy2_ur; Vx2_ur];
            Vy2= Vt2(1); Vx2= Vt2(2);
            Vy3= Vy3_ur; Vx3= Vx3_ur;
            Vy4= Vy4_ur; Vx4= Vx4_ur;

            %% Slip Angles
            alpha1_aux= this.slip_angle(Vx1,Vy1); %(-atan(Vy1/Vx1)-alpha1)*(Vx1/this.sig);
            alpha2_aux= this.slip_angle(Vx2,Vy2);%(-atan(Vy2/Vx2)-alpha2)*(Vx2/this.sig);
            alpha3_aux= this.slip_angle(Vx3,Vy3);%(-atan(Vy3/Vx3)-alpha3)*(Vx3/this.sig);
            alpha4_aux= this.slip_angle(Vx4,Vy4);%(-atan(Vy4/Vx4)-alpha4)*(Vx4/this.sig);

            alphadot1= -Vx1/this.sig*(alpha1+alpha1_aux);
            alphadot2= -Vx2/this.sig*(alpha2+alpha2_aux);
            alphadot3= -Vx3/this.sig*(alpha3+alpha3_aux);
            alphadot4= -Vx4/this.sig*(alpha4+alpha4_aux);

            %% Slip Ratios
            kappa1= this.slip_ratio(Vx1,omega1);%((this.Rw*omega1)-Vx1)/Vx1;
            kappa2= this.slip_ratio(Vx2,omega2);%((this.Rw*omega2)-Vx2)/Vx2;
            kappa3= this.slip_ratio(Vx3,omega3);%((this.Rw*omega3)-Vx3)/Vx3;
            kappa4= this.slip_ratio(Vx4,omega4);%((this.Rw*omega4)-Vx4)/Vx4;

            %% Magic Formula
            
            [Fx1_ur, Fy1_ur] = this.pacejka_model(alpha1, kappa1, Fz1, 1);  % front
            [Fx2_ur, Fy2_ur] = this.pacejka_model(alpha2, kappa2, Fz2, 1);  % front
            [Fx3, Fy3] =       this.pacejka_model(alpha3, kappa3, Fz3, 0);  % rear
            [Fx4, Fy4] =       this.pacejka_model(alpha4, kappa4, Fz4, 0);  % rear

            Fxy1 = rot1*[Fx1_ur; Fy1_ur];
            Fx1 = Fxy1(1); Fy1 = Fxy1(2);
            Fxy2 = rot1*[Fx2_ur; Fy2_ur];
            Fx2 = Fxy2(1); Fy2 = Fxy2(2);

            %% Omega Dots
            omegadot1= (Tf/2-(this.Rw*Fx1))/this.Iw;
            omegadot2= (Tf/2-(this.Rw*Fx2))/this.Iw;
            omegadot3= (Tr/2-(this.Rw*Fx3))/this.Iw;
            omegadot4= (Tr/2-(this.Rw*Fx4))/this.Iw;

            %% Net Forces and Moments
            Fx= Fx1+Fx2+Fx3+Fx4;
            Fy= Fy1+Fy2+Fy3+Fy4;
            Mz= this.l_f*(Fy1+Fy2) + this.w*(Fx2-Fx1) - this.l_r*(Fy3+Fy4) - this.w*(Fx4+Fx3);

            %% Euler Angles' Angular Accelerations

            num1= -(tau_phi) + this.h*(Fy*cos(phi)*cos(theta)+this.m*this.g*sin(phi)) + ...
                psidot*(this.Iyy-this.Izz)*(psidot*sin(phi)*cos(phi)*cos(theta)+ ...
                phidot*sin(theta)*sin(phi)*cos(phi)) + psidot*thetadot*(cos(phi)^2*this.Iyy+sin(phi)^2*this.Izz);

            den1= (this.Ixx*cos(theta)^2+this.Iyy*sin(theta)^2*sin(phi)^2+this.Izz*sin(theta)^2*cos(phi)^2);
            phiddot= num1/den1;

            num2= -(tau_theta) + this.h*(this.m*this.g*sin(theta)*cos(phi)-Fx*cos(theta)*cos(phi)) + ...
                psidot*(psidot*sin(theta)*cos(theta)*(this.Ixx-this.Iyy+cos(phi)^2*(this.Iyy-this.Izz))- ...
                phidot*(cos(theta)^2*this.Ixx+sin(phi)^2*sin(theta)^2*this.Iyy+sin(theta)^2*cos(phi)^2*this.Izz)- ...
                thetadot*(sin(theta)*sin(phi)*cos(phi)*(this.Iyy-this.Izz)));
            den2= (this.Iyy*cos(phi)^2+this.Izz*sin(phi)^2);
            thetaddot= num2/den2;

            num3= Mz - this.h*(Fx*sin(phi)+Fy*sin(theta)*cos(phi));
            den3= (this.Ixx*sin(theta)^2+cos(theta)^2*(this.Iyy*sin(phi)^2+this.Izz*cos(phi)^2));

            psiddot= num3/den3;
            %% Net Linear Accelerations
            Vxdot= Vy*psidot + this.h*(sin(theta)*cos(phi)*(psidot^2+phidot^2+thetadot^2)- ...
                sin(phi)*psiddot-2*cos(phi)*phidot*psidot-cos(theta)*cos(phi)*thetaddot+ ...
                2*cos(theta)*sin(phi)*thetadot*phidot+sin(theta)*sin(phi)*phiddot)+Fx/this.m;

            Vydot= -Vx*psidot + this.h*(-sin(theta)*cos(phi)*psiddot-sin(phi)*psidot^2- ...
                2*cos(theta)*cos(phi)*thetadot*psidot+ ...
                sin(theta)*sin(phi)*phidot*psidot-sin(phi)*phidot^2+cos(phi)*phiddot) + Fy/this.m;

           
            % 18 States (in order) - Px,Py,Psi,Vx,Vy,PsiDot,Theta,ThetaDot, Phi, ...
            % PhiDot, omega1,2,3,4, alpha1,2,3,4
            rot= [cos(psi) -sin(psi);
                sin(psi)  cos(psi)];
            Vxy= rot*[Vx;Vy];
            Vx= Vxy(1); Vy= Vxy(2);
            dx= [Vx;Vy;psidot;Vxdot;Vydot;psiddot;thetadot;thetaddot;phidot;phiddot; ...
                omegadot1;omegadot2;omegadot3;omegadot4;alphadot1;alphadot2;alphadot3;alphadot4];
                     
                statesDot= dx;
                forces.Fx= [Fx1;Fx2;Fx3;Fx4];
                forces.Fy= [Fy1;Fy2;Fy3;Fy4];
                forces.Fz= [Fz1;Fz2;Fz3;Fz4];
        end
        function [states_next,statesDot,forces]= PropagateVehicleDynamics(this,states,inputs)
            if this.Ts<0.01
                % Euler
                [statesDot,forces]=VehicleDynamics(this,states,inputs);
                states_next= statesDot*this.Ts+states;
            else
                % RK4
                [k1,forces] = VehicleDynamics(this,states,inputs);
                [k2,~] = VehicleDynamics(this,state + this.Ts/2*k1, inputs);
                [k3,~] = VehicleDynamics(this,state + this.Ts/2*k2, inputs);
                [k4,~] = VehicleDynamics(this,state + this.Ts*k3, inputs);
                statesDot= 1/6*(k1 +2*k2 +2*k3 +k4);
                states_next=states + this.Ts*statesDot;
            end
        end
        function [Fl,Fc] = pacejka_model(this,alpha, s, Fz, front)
            s0 = s;
            alpha0 = alpha;
            
            mu_x_f = this.tire_params(1);
            Bx_f = this.tire_params(3);
            Cx_f = this.tire_params(5);
            Ex_f = this.tire_params(7);

            mu_y_f = this.tire_params(2);
            By_f = this.tire_params(4);
            Cy_f = this.tire_params(6);
            Ey_f = this.tire_params(8);

            mu_x_r = this.tire_params(9);
            Bx_r = this.tire_params(11);
            Cx_r = this.tire_params(13);
            Ex_r = this.tire_params(15);

            mu_y_r = this.tire_params(10);
            By_r = this.tire_params(12);
            Cy_r = this.tire_params(14);
            Ey_r = this.tire_params(16);

            if front == 1
                mu_x = mu_x_f;
                Bx = Bx_f;
                Cx = Cx_f;
                Ex = Ex_f;

                mu_y = mu_y_f;
                By = By_f;
                Cy = Cy_f;
                Ey = Ey_f;
            else
                mu_x = mu_x_r;
                Bx = Bx_r;
                Cx = Cx_r;
                Ex = Ex_r;

                mu_y = mu_y_r;
                By = By_r;
                Cy = Cy_r;
                Ey = Ey_r;
            end

            % NOMINAL Longitudinal tire force
            Fx0 = mu_x*Fz*sin(Cx*atan(Bx*(1-Ex)*s0+Ex*atan(Bx*s0)));

            % NOMINAL Lateral tire force
            Fy0 = mu_y*Fz*sin(Cy*atan(By*(1-Ey)*alpha0+Ey*atan(By*alpha0)));

            Fl = Fx0;

            Fc = Fy0*sqrt(1-(Fx0/(mu_x*Fz))^2+this.epsilon); % Lon-Lat force coupling

        end
        function [ out ] = sign_sigmoid(this,in)
            out = -1+2/(1+exp(-this.scale*in));
        end
        function [ alpha ] = slip_angle(this,vl, vc)
            alpha = atan(this.sign_sigmoid(vl)*vc/sqrt(vl.^2+this.epsilon));
            % alpha = atan(vc/vl);
        end
        function [ slip ] = slip_ratio(this,v, omega)
            slip = this.sign_sigmoid(v)*(this.Rw.*omega-v)./(sqrt(v.^2+this.epsilon));
            % slip = (r.*omega-v)./(v);

        end
    end
end
