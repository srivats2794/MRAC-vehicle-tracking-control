ctrl_exec_freq= round(controlTs/simTs);
j=1;

for i=1:floor(Tsim/simTs)
    % Localization
    X= X_pl_Record(1,i);  Y= X_pl_Record(2,i); psi= X_pl_Record(3,i);
    xDot= X_pl_Record(4,i); yDot= X_pl_Record(5,i); psiDot= X_pl_Record(6,i);
    xDDot= XDot_pl_Record(4,i);yDDot = XDot_pl_Record(5,i);
    if(rem(i,ctrl_exec_freq)==1)
        % Reference
        X_ref= ref.x_ref(j+1); Y_ref= ref.y_ref(j+1); psi_ref= ref.psi_ref(j+1);
        v_ref= ref.v_ref(j+1:j+lon_mpc.N+1);
        kappa_ref= ref.curvature_ref(j+1:j+lon_mpc.N+1);

        v_ref_curr= ref.v_ref(j); kappa_ref_curr= ref.curvature_ref(j);

        delta_cmd_og = atan(plant.l*kappa_ref_curr);
            
        X_MRAC_vec(:,j)=calc_error_states(X,Y,psi,psiDot,xDot,yDot,X_ref,Y_ref,psi_ref,kappa_ref(1));
        if j==1
            X_MRAC_bar_vec(:,j)= X_MRAC_vec(:,j);
        end
        err_mrac_vec(:,j)= X_MRAC_vec(:,j)-X_MRAC_bar_vec(:,j);
        tStart=cputime;
        delta_cmd= mrac.Ky'*X_MRAC_vec(:,j)+mrac.Ke'*err_mrac_vec(:,j);
        delta_cmd= delta_cmd+mrac.Kdel*delta_cmd_og;
        time_mrac1_end=cputime - tStart;
        gains_MRAC_vec(:,j)= [mrac.Kdel;mrac.Ky;mrac.Ke];

        
        % Update Lon MPC problem
        upLonParams= updateLonMpc(xDot,xDDot,v_ref(1:lon_mpc.N+1),yDot,psiDot,lon_mpc);
        lon_mpc.solver.update('q', upLonParams.q, 'l', upLonParams.l, 'u', upLonParams.u);
        
        lon_res= lon_mpc.solver.solve();
        
        if ~strcmp(lon_res.info.status, 'solved')
            error('Lon OSQP did not solve the problem!')
        end
        run_time_lon(j)= lon_res.info.run_time;
        
        ax_cmd= lon_res.x((lon_mpc.N+1)*lon_mpc.nx+1:(lon_mpc.N+1)*lon_mpc.nx+lon_mpc.nu);
           
        tau_f_cmd= ((lon_mpc.sys_params.m*ax_cmd)/2)*lon_mpc.sys_params.Rw;
        tau_r_cmd= tau_f_cmd;
        tStart = cputime;
        X_MRAC_bar_vec(:,j+1)= propagate_error_dynamics(lat_control_params,xDot,controlTs, ...
            X_MRAC_vec(:,j),v_ref_curr*kappa_ref_curr,delta_cmd_og);
        
        mrac= mrac.updateMRACgains(lat_control_params,...
            X_MRAC_vec(:,j),err_mrac_vec(:,j), ...
            xDot,delta_cmd);
        time_mrac2_end=cputime - tStart;
        run_time_lat(j)=time_mrac2_end+time_mrac1_end;
        j=j+1;    
    end

    U_Record(:,i)=[ax_cmd;tau_f_cmd;tau_r_cmd;delta_cmd];
    [X_pl_Record(:,i+1),XDot_pl_Record(:,i+1),~]= ...
        plant.PropagateVehicleDynamics(X_pl_Record(:,i),U_Record(2:4,i));    
end