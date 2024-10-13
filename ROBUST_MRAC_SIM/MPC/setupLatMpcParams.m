function [params]=setupLatMpcParams(vehicle,control_Ts)
    % A_sys,B_sys,N,Q,R,nx,nu,nd,QN,Ts,x_min,x_max,u_min,u_max
    params.N=50;
    params.Q= diag([5; 0.1; 5; 1;0.1]);
    params.R= 10*eye(1);
    params.nx=5;
    params.nu=1;
   
    params.QN= params.Q;
    params.Ts= control_Ts;
    params.x_min= -inf*ones(params.nx,1);
    params.x_max= inf*ones(params.nx,1);
    params.u_min= -deg2rad(45);
    params.u_max= deg2rad(45);
    
    m = vehicle.m;
    I_z= vehicle.Izz;
    l_f= vehicle.l_f;
    l_r= vehicle.l_r;

    params.sys_params.m = m;
    params.sys_params.I_z= I_z;
    params.sys_params.l_f= l_f;
    params.sys_params.l_r= l_r;

    params.sys_params.C_f= m*(l_f/(l_f+l_r))*0.50*0.165*57.29578*vehicle.g;
    params.sys_params.C_r= m*(l_r/(l_f+l_r))*0.5*0.165*57.29578*vehicle.g;
    
    C_f= params.sys_params.C_f; C_r= params.sys_params.C_r;

    A =   [0                  1                                            0                                     0                                                              ;...
        0  -(2*(C_f+C_r)/(m*3))                     2*(C_f+C_r)/m             -2*(C_f*l_f-C_r*l_r)/(m*3)                     ;...
        0                  0                                            0                                     1                                                              ;...
        0  -(2*(C_f*l_f-C_r*l_r)/(I_z*3))    2*(C_f*l_f-C_r*l_r)/I_z    -(2*(C_f*l_f^2+C_r*l_r^2)/(I_z*3))   ];

    B            = [0; 2*C_f/m  ; 0;  2*C_f*l_f/I_z];


            A_bar        = [A B ; zeros(1,4) -1/control_Ts];
            B_bar        = [zeros(4,1); 1/control_Ts];

            %%%%%%% discrete time model %%%%%%%
            % Ad=eye(5)+Ac*dt_control;
            % Bd=dt_control*Bc;

          C_bar                  =[1 0 0 0 0];
          D_bar                  =0;
          sys_c                   =ss(A_bar,B_bar,C_bar,D_bar);
          opt                     =c2dOptions('Method','zoh');
          sys_d                   =c2d(sys_c,control_Ts,opt);
         
    params.sys_params.A_sys = sys_d.A;
    params.sys_params.B_sys= sys_d.B;
end
