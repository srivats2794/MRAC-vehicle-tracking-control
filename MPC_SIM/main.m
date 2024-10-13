ctrl_exec_freq= round(controlTs/simTs);
j=1;
err_prev=zeros(4,1);
delta_st=0;

for i=1:floor(Tsim/simTs)
    % Localization
    X= X_pl_Record(1,i);  Y= X_pl_Record(2,i); psi= X_pl_Record(3,i);
    xDot= X_pl_Record(4,i); yDot= X_pl_Record(5,i); psiDot= X_pl_Record(6,i);
    xDDot= XDot_pl_Record(4,i);yDDot = XDot_pl_Record(5,i);
    
    if(rem(i,ctrl_exec_freq)==1)
        % Reference
        X_ref= ref.x_ref(j+1); Y_ref= ref.y_ref(j+1); psi_ref= ref.psi_ref(j+1);
        v_ref= ref.v_ref(j+1:j+max(lat_mpc.N,lon_mpc.N)+1);
        kappa_ref= ref.curvature_ref(j+1:j+max(lat_mpc.N,lon_mpc.N)+1);

        v_ref_curr= ref.v_ref(j); kappa_ref_curr= ref.curvature_ref(j);

      
        error_states= calc_error_states(X,Y,psi,psiDot,xDot,yDot,X_ref,Y_ref,psi_ref,kappa_ref(1)); 
       
        
       
         % Update Lat MPC problem
        upLatParams= updateLatMpc(v_ref(1:lat_mpc.N+1).*kappa_ref(1:lat_mpc.N+1), ...
                                  xDot,[error_states;delta_st],lat_mpc,kappa_ref(1),0); 
        lat_mpc.solver.update('Px', nonzeros(upLatParams.P),'Ax', nonzeros(upLatParams.A),'q',upLatParams.q,'l', upLatParams.l, 'u', upLatParams.u);

        % Update Lon MPC problem
        upLonParams= updateLonMpc(xDot,xDDot,v_ref(1:lon_mpc.N+1),yDot,psiDot,lon_mpc);
        lon_mpc.solver.update('q', upLonParams.q, 'l', upLonParams.l, 'u', upLonParams.u);
        
        lon_res= lon_mpc.solver.solve();
        lat_res= lat_mpc.solver.solve();
        
        if ~strcmp(lon_res.info.status, 'solved')
            error('Lon OSQP did not solve the problem!')
        end
        run_time_lon(j)= lon_res.info.run_time;
        run_time_lat(j)= lat_res.info.run_time;
        ax_cmd= lon_res.x((lon_mpc.N+1)*lon_mpc.nx+1:(lon_mpc.N+1)*lon_mpc.nx+lon_mpc.nu);
        delta_cmd = lat_res.x((lat_mpc.N+1)*lat_mpc.nx+1:(lat_mpc.N+1)*lat_mpc.nx+lat_mpc.nu);

        tau_f_cmd= ((lon_mpc.sys_params.m*ax_cmd)/2)*lon_mpc.sys_params.Rw;
        tau_r_cmd= tau_f_cmd;
        j=j+1;    
    end


    U_Record(:,i)=[ax_cmd;tau_f_cmd;tau_r_cmd;delta_cmd];

    [X_pl_Record(:,i+1),XDot_pl_Record(:,i+1),~]= ...
        plant.PropagateVehicleDynamics(X_pl_Record(:,i),U_Record(2:4,i));    

    delta_st=delta_cmd;
end
