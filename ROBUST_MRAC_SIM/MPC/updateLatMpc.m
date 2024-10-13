function params=updateLatMpc(psiDot_des,vx,error_states,lat_mpc,kappa)
  
    lat= lat_mpc.sys_params;
    C_r= lat.C_r; 
    
    Ac =   [0                  1                                            0                                     0                                                              ;...
        0  -(2*(lat.C_f+C_r)/(lat.m*vx))                     2*(lat.C_f+C_r)/lat.m             -2*(lat.C_f*lat.l_f-C_r*lat.l_r)/(lat.m*vx)                     ;...
        0                  0                                            0                                     1                                                              ;...
        0  -(2*(lat.C_f*lat.l_f-C_r*lat.l_r)/(lat.I_z*vx))    2*(lat.C_f*lat.l_f-C_r*lat.l_r)/lat.I_z    -(2*(lat.C_f*lat.l_f^2+C_r*lat.l_r^2)/(lat.I_z*vx))   ];
    
    Bc            = [0; 2*lat.C_f/lat.m  ; 0;  2*lat.C_f*lat.l_f/lat.I_z];
    
    %%%%%%% Augmented model to include DBW model %%%%%%%
    
    Ac_bar        = [Ac Bc ; zeros(1,4) -1/lat_mpc.Ts];
    Bc_bar        = [zeros(4,1); 1/lat_mpc.Ts];
    
    %%%%%%% discrete time model %%%%%%%
    % Ad=eye(5)+Ac*dt_control;
    % Bd=dt_control*Bc;
    
    Cc_bar                  =[1 0 0 0 0];
    Dc_bar                  =0;
    sys_c                   =ss(Ac_bar,Bc_bar,Cc_bar,Dc_bar);
    opt                     =c2dOptions('Method','zoh');
    sys_d                   =c2d(sys_c,lat_mpc.Ts,opt);
    Ad                      = sys_d.A;
    Bd                      = sys_d.B;
    
    %%%%%%% desired yaw rate effect %%%%%%%
    Gc              = [0  ;  -vx-(2*(lat.C_f*lat.l_f-C_r*lat.l_r))/(lat.m*vx)  ;  0  ; -(2*(lat.C_f*lat.l_f^2+C_r*lat.l_r^2))/(lat.I_z*vx); 0];
    Gd              =lat_mpc.Ts*Gc;
    z_new           = Gd * (vx*kappa);
    
    params.Ad= Ad; params.Bd= Bd; params.Gd= Gd;
    %%%%%%% include history of psiDotDes %%%%%%%
    B_psi_hist=repmat(Gd,1,length(psiDot_des(2:end))).*repmat(psiDot_des(2:end).',length(Gd),1);
    B_psi_hist=B_psi_hist(:);
    
    
    umin             = lat_mpc.u_min;
    umax             = lat_mpc.u_max;
    xmin             = lat_mpc.x_min;
    xmax             = lat_mpc.x_max;
    
    xr= zeros(lat_mpc.nx,1);
    
    
    params.P             = blkdiag( kron(speye(lat_mpc.N), lat_mpc.Q), lat_mpc.QN, kron(speye(lat_mpc.N), lat_mpc.R) );
    
    params.q             = [repmat(-lat_mpc.Q*xr, lat_mpc.N, 1); -lat_mpc.QN*xr; zeros(lat_mpc.N*lat_mpc.nu, 1)];
    
    Ax            = kron(speye(lat_mpc.N+1), -speye(lat_mpc.nx)) + kron(sparse(diag(ones(lat_mpc.N, 1), -1)), Ad);
    Bu            = kron([sparse(1, lat_mpc.N); speye(lat_mpc.N)], Bd);
    Aeq           = [Ax, Bu];
    leq           = [-error_states-z_new; -B_psi_hist];
    ueq           = leq;
    
    Aineq         = speye((lat_mpc.N+1)*lat_mpc.nx + lat_mpc.N*lat_mpc.nu);
    lineq         = [repmat(xmin, lat_mpc.N+1, 1); repmat(umin, lat_mpc.N, 1)];
    uineq         = [repmat(xmax, lat_mpc.N+1, 1); repmat(umax, lat_mpc.N, 1)];
    
    params.A             = [Aeq; Aineq];
    params.l             = [leq; lineq];
    params.u             = [ueq; uineq];
end
