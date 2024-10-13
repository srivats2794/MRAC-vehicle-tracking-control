function params = setupMRAC(vehicle,Vx,Ts)

    m= vehicle.m;
    C_f= vehicle.C_f;
    C_r= vehicle.C_r;
    I_z = vehicle.I_z;
    l_f= vehicle.l_f;
    l_r= vehicle.l_r;

    Ac =   [0                  1                                            0                                     0                                                              ;...
        0  -(2*(C_f+C_r)/(m*Vx))                     2*(C_f+C_r)/m             -2*(C_f*l_f-C_r*l_r)/(m*Vx)                     ;...
        0                  0                                            0                                     1                                                              ;...
        0  -(2*(C_f*l_f-C_r*l_r)/(I_z*Vx))    2*(C_f*l_f-C_r*l_r)/I_z    -(2*(C_f*l_f^2+C_r*l_r^2)/(I_z*Vx))   ];
    
    Bc            = [0; 2*C_f/m  ; 0;  2*C_f*l_f/I_z];

    Gc= [0  ;  -Vx-(2*(C_f*l_f-C_r*l_r))/(m*Vx)  ;  0  ; -(2*(C_f*l_f^2+C_r*l_r^2))/(I_z*Vx)];

    params.A=Ac;
    params.B=Bc;
    factor= 5e-2*0.02;
    params.G= Gc*Ts;

    params.gamma_st  = factor*diag([2;0.5;2;0.5]);
    params.gamma_err = factor*diag([2;0.5;2;0.5]);
    params.gamma_in = factor*0.001;

    params.lambda= 4e-3;

    params.Q= 0.1*eye(4);
    params.R= 0.1*eye(1);

    params.Kdel=1.00219175876205;
    params.Ky=[-0.0611432181370346;...
        -0.0167388178135198;...
        -0.00252976064137874;...
        0.0187520530715997];
    params.Ke= [-0.000588284674229885;
        -0.00113296623619705;
        -8.56176009342476e-06;
        -0.00139190878782081];

    params.Ts= Ts;
end

