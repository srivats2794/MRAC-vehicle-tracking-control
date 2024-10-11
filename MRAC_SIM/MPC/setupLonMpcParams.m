function params=setupLonMpcParams(plant,control_Ts)
    % A_sys,B_sys,N,Q,R,nx,nu,nd,QN,Ts,x_min,x_max,u_min,u_max
    params.N=100;
    params.Q= diag([0.5; 0]);
    params.R= 0.1*eye(1);
    params.nx=2;
    params.nu=1;
  
    params.QN= diag([0.1; 0]);
    params.Ts= control_Ts;
    params.x_min= -inf*ones(params.nx,1);
    params.x_max= inf*ones(params.nx,1);
    params.u_min= -5;
    params.u_max= 5;
    params.sys_params.m= plant.m;
    params.sys_params.Rw= plant.Rw;
    Ac    = [0 1; 0 -1/control_Ts];
    Bc    = [0; 1/control_Ts];
    Cc    = [1 0];
    Dc    = 0;

    [params.sys_params.A_sys,params.sys_params.B_sys,~,~]=ssdata(c2d(ss(Ac,Bc,Cc,Dc),control_Ts));
end
