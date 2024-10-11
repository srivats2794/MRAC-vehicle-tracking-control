function ref= ReferenceGenerator(Ts,plotter,scenario)    

        if scenario==1    
            load("ReferenceRaw_CityScene.mat"); 
            load("lat_error_oval.mat");
        elseif scenario==2
            load("ReferenceRaw_Figure8.mat");
            load("lat_error_figure8_60.mat");
        else
            load("ReferenceRaw_Figure8_LowSpeed_Modified.mat");
            load("lat_error_figure8_45.mat");
        end
        Ts_data= 1/50; % Preferred sample time for reference
        
        x_ref= out.Reference.signal_264__X_Position_ReferencePath_global__m_.Data(:,1);
        y_ref= out.Reference.signal_265__Y_Position_ReferencePath_global__m_.Data(:,1);
        v_ref= out.Reference.signal_253__Target_Speed_For_Default_Controller__km_h_.Data(:,1);
        
        v_ref= v_ref*0.277778; % Converting to m/s
        v_ref= smooth(v_ref,0.05,'loess'); % Smoothen velocity
        
        theta_act= deg2rad(out.KinematicState.signal_3D_Rotations_RA.signal_80__Yaw_Angle__deg_.Data(:,1));
        t_ref=out.Time.Time;
        Tsim= t_ref(end);
        t_vec= (0:Ts:Tsim)';
        curvature_ref= out.Reference.signal_255__Curvature_Reference_Default_Controller__m__1_.Data(:,1);
        curvature_ref= smooth(curvature_ref,0.05,'loess'); % Smoothen Curvature
        
        a_ref= diff(v_ref)./diff(t_ref);
        a_ref= smooth(a_ref,0.01,'loess');
        a_ref(end+1)=a_ref(end); % Smoothen Acceleration
        
        theta_ref = unwrap(atan2(y_ref,x_ref));
        
        ref.x_ref=makima(t_ref,x_ref,t_vec);
        ref.y_ref=makima(t_ref,y_ref,t_vec);
        ref.psi_ref=makima(t_ref,theta_act,t_vec);
        ref.v_ref= makima(t_ref,v_ref,t_vec);
        ref.curvature_ref=makima(t_ref,curvature_ref,t_vec);
        ref.a_ref=makima(t_ref,a_ref,t_vec);
        ref.t_ref=t_vec;
        
        if plotter
            figure(1)
            nexttile
            plot(x_ref,y_ref);
            hold on
            plot(ref.x_ref,ref.y_ref);
            hold off
            title("Reference Path")
            nexttile
            plot(t_ref,a_ref);
            hold on
            plot(ref.t_ref,ref.a_ref);
            hold off
            title("Reference Acceleration")
            nexttile
            plot(t_ref,curvature_ref);
            hold on
            plot(ref.t_ref,ref.curvature_ref);
            hold off
            title("Reference Curvature")
            nexttile
            plot(t_ref,v_ref);
            hold on
            plot(ref.t_ref,ref.v_ref);
            hold off
            title("Reference Speed")
            nexttile
            plot(t_ref,theta_act);
            hold on
            plot(ref.t_ref,ref.psi_ref);
            hold off
            title("Reference Yaw")
        end
    
        ref.x_ref(end+1:end+200)= ref.x_ref(end);
        ref.y_ref(end+1:end+200)= ref.y_ref(end);
        ref.psi_ref(end+1:end+200)= ref.psi_ref(end);
        ref.v_ref(end+1:end+200)= ref.v_ref(end);
        ref.curvature_ref(end+1:end+200)= ref.curvature_ref(end);
        ref.a_ref(end+1:end+200)= 0;
        ref.lat_error=lat_error;
end
