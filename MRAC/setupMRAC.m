function params = setupMRAC(vehicle,Vx,Ts)
    
    Ac =   [0                  1                                            0                                     0                                                              ;...
        0  -(2*(vehicle.C_f+vehicle.C_r)/(vehicle.m*Vx))                     2*(vehicle.C_f+vehicle.C_r)/vehicle.m             -2*(vehicle.C_f*vehicle.l_f-vehicle.C_r*vehicle.l_r)/(vehicle.m*Vx)                     ;...
        0                  0                                            0                                     1                                                              ;...
        0  -(2*(vehicle.C_f*vehicle.l_f-vehicle.C_r*vehicle.l_r)/(vehicle.I_z*Vx))    2*(vehicle.C_f*vehicle.l_f-vehicle.C_r*vehicle.l_r)/vehicle.I_z    -(2*(vehicle.C_f*vehicle.l_f^2+vehicle.C_r*vehicle.l_r^2)/(vehicle.I_z*Vx))   ];
    
    Bc            = [0; 2*vehicle.C_f/vehicle.m  ; 0;  2*vehicle.C_f*vehicle.l_f/vehicle.I_z];


    Cc= eye(4);

    Dc= zeros(4,1);

    Gc= [0  ;  -Vx-(2*(vehicle.C_f*vehicle.l_f-vehicle.C_r*vehicle.l_r))/(vehicle.m*Vx)  ;  0  ; -(2*(vehicle.C_f*vehicle.l_f^2+vehicle.C_r*vehicle.l_r^2))/(vehicle.I_z*Vx)];

    [params.A,params.B,~,~] = ssdata(c2d(ss(Ac,Bc,Cc,Dc),Ts));
    factor= 5e-2;%5e-2;
    params.G= Gc*Ts;
    %   params.gamma_st  = factor*diag([1;0.05;1;0.05]);
    % params.gamma_err = factor*diag([1;0.05;1;0.05]);
    % params.gamma_in = factor*1;
    params.gamma_st  = factor*diag([2;0.5;2;0.5]);
    params.gamma_err = factor*diag([2;0.5;2;0.5]);
    params.gamma_in = factor*0.001;

    params.lambda= 4e-3;

    params.Q= diag([1; 0.1; 5; 1]);
    R= 10*eye(1);

    [params.Ky0,~,~]= dlqr(params.A,params.B,params.Q,R);
    params.Q= eye(4);
    % 
%     params.Kdel=0.1;
%     params.Ky=[-0.0683558709676514;...
%         -0.0361051244489816;...
%         -0.00645418713886534;...
%         0.00381771957682497;];
%     params.Ke=[-0.000223888751060068;...
%         -0.00204897131565473;...
%         1.29252945555274e-06;...
%         -0.000513538777019688];
%     params.Kdel=0.1;
%     params.Ky=[-0.0152455883453572; ...
%     -0.0214367063455597; ....
%     -0.00468831853491997; ....
%     0.00168559638164822];
%     params.Ke=[-0.000140022442615681;
%     -0.000576762491070962;
%     5.29166291288736e-08;
%     -6.72385202858717e-05];
    params.Kdel=1;
   params.Ky= [0;
        0;
        0;
        0];
    params.Ke= [0;
        0;
        0;
        0];
    % 
    % params.Kdel=1.00219175876205;
    % params.Ky=[-0.0611432181370346;...
    %     -0.0167388178135198;...
    %     -0.00252976064137874;...
    %     0.0187520530715997];
    % params.Ke= [-0.000588284674229885;
    %     -0.00113296623619705;
    %     -8.56176009342476e-06;
    %     -0.00139190878782081];

    params.Kdel=1.00085523370669;
    params.Ky=[-0.0532137172376177;...
        -0.00223015239564354;...
        -0.000697438686972409;...
        0.00150635617167024];
    params.Ke= [-0.000248743354577025;
        -0.000166581942786853;
        1.63838942320971e-07;
        -0.000118398860472309];

    % params.Kdel=1.00045044519116;
    % params.Ky=[-0.0273543598983924;...
    %     -0.00281437662013109;...
    %     -0.000680848442228389;...
    %     0.000427810781425014];
    % params.Ke= [-0.000134984646001675;...
    %     -0.000130212269596431;...
    %     -2.24278253097235e-07;...
    %     -3.41011098563302e-05];

    params.Ts= Ts;
end

