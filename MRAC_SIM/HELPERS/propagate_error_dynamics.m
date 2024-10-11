function error_bar= propagate_error_dynamics(vehicle,Vx,Ts,error_curr,thetaDot_des,delta_ref)
Ac =   [0                  1                                            0                                     0                                                              ;...
                0  -(2*(vehicle.C_f+vehicle.C_r)/(vehicle.m*Vx))                     2*(vehicle.C_f+vehicle.C_r)/vehicle.m             -2*(vehicle.C_f*vehicle.l_f-vehicle.C_r*vehicle.l_r)/(vehicle.m*Vx)                     ;...
                0                  0                                            0                                     1                                                              ;...
                0  -(2*(vehicle.C_f*vehicle.l_f-vehicle.C_r*vehicle.l_r)/(vehicle.I_z*Vx))    2*(vehicle.C_f*vehicle.l_f-vehicle.C_r*vehicle.l_r)/vehicle.I_z    -(2*(vehicle.C_f*vehicle.l_f^2+vehicle.C_r*vehicle.l_r^2)/(vehicle.I_z*Vx))   ];

            Bc            = [0; 2*vehicle.C_f/vehicle.m  ; 0;  2*vehicle.C_f*vehicle.l_f/vehicle.I_z];


            Cc= eye(4);

            Dc= zeros(4,1);

            Gc= [0  ;  -Vx-(2*(vehicle.C_f*vehicle.l_f-vehicle.C_r*vehicle.l_r))/(vehicle.m*Vx)  ;  0  ; -(2*(vehicle.C_f*vehicle.l_f^2+vehicle.C_r*vehicle.l_r^2))/(vehicle.I_z*Vx)];

           

            G = Gc*Ts;

            [A,B,~,~] = ssdata(c2d(ss(Ac,Bc,Cc,Dc),Ts));

            error_bar= A*error_curr+B*delta_ref+G*thetaDot_des;
end