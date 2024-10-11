classdef MRAC_CONTROLLER
    % A class implementation that encapsulates all the MRAC controller properties and methods 
    
    
    properties
       gamma_st; 
       gamma_err;
       gamma_in; 
       Q 
       A
       B
       G
       P
       lambda;
       Kdel; Ky;Ke; Ky0;
       Ts
    end
    
    methods
        function obj = MRAC_CONTROLLER(params)
            %MRAC_CONTROLLER Construct an instance of this class
            %   Detailed explanation goes here
            obj.gamma_st = params.gamma_st;
            obj.gamma_err= params.gamma_err;
            obj.gamma_in= params.gamma_in;
            obj.Q= params.Q;
            obj.A= params.A;
            obj.B= params.B;
            obj.P=eye(4);
            obj.lambda=params.lambda;
            obj.Ts= params.Ts;
            obj.Ke= params.Ke;
            obj.Kdel= params.Kdel;
            obj.Ky= params.Ky;
            obj.Ky0 = params.Ky0;
            obj.G= params.G;
        end
        
        function this = updateMRACgains(this,vehicle,meas,err,Vx,in)


            Ac =   [0                  1                                            0                                     0                                                              ;...
                0  -(2*(vehicle.C_f+vehicle.C_r)/(vehicle.m*Vx))                     2*(vehicle.C_f+vehicle.C_r)/vehicle.m             -2*(vehicle.C_f*vehicle.l_f-vehicle.C_r*vehicle.l_r)/(vehicle.m*Vx)                     ;...
                0                  0                                            0                                     1                                                              ;...
                0  -(2*(vehicle.C_f*vehicle.l_f-vehicle.C_r*vehicle.l_r)/(vehicle.I_z*Vx))    2*(vehicle.C_f*vehicle.l_f-vehicle.C_r*vehicle.l_r)/vehicle.I_z    -(2*(vehicle.C_f*vehicle.l_f^2+vehicle.C_r*vehicle.l_r^2)/(vehicle.I_z*Vx))   ];

            Bc            = [0; 2*vehicle.C_f/vehicle.m  ; 0;  2*vehicle.C_f*vehicle.l_f/vehicle.I_z];


            Cc= eye(4);

            Dc= zeros(4,1);

            Gc= [0  ;  -Vx-(2*(vehicle.C_f*vehicle.l_f-vehicle.C_r*vehicle.l_r))/(vehicle.m*Vx)  ;  0  ; -(2*(vehicle.C_f*vehicle.l_f^2+vehicle.C_r*vehicle.l_r^2))/(vehicle.I_z*Vx)];

           

            this.G = Gc*this.Ts;

            [this.A,this.B,~,~] = ssdata(c2d(ss(Ac,Bc,Cc,Dc),this.Ts));
            % this.A=Ac;
            % this.B=Bc;
            %this.P= dlyap((this.A-this.B*this.Ky0),this.Q);

            Ky_dot= this.gamma_st*meas*err'*this.P*this.B;
            Kdel_dot= this.gamma_in*in*err'*this.P*this.B;
            Ke_dot= -this.gamma_err*(err*err')*this.P*this.B;

            this.Kdel= Kdel_dot*this.Ts+this.Kdel;
            this.Ky= Ky_dot*this.Ts+this.Ky;
            this.Ke=Ke_dot*this.Ts+this.Ke;
        end

    end
end

