close all;
len= length(ref.t_ref);
 linewidth=2;  
    figure(1)
    %tiledlayout("vertical")
    nexttile
    
    plot(ref.t_ref,ref.v_ref(1:len),'-k','LineWidth',linewidth);
    set(gca, 'FontSize', 14);
    hold on
    plot(t',X_pl_Record(4,:),'--r','LineWidth',linewidth);
    set(gca, 'FontSize', 14);
    hold off
    title("Velocity Tracking",FontSize=15)
    legend('Reference','Feedback',FontSize=15);
    xlabel('Time (s)',FontSize=15) 
    ylabel('Velocity (m/s)',FontSize=15) 
    
    nexttile
    plot(ref.x_ref(1:len),ref.y_ref(1:len),'-k','LineWidth',linewidth);
    set(gca, 'FontSize', 14);
    hold on
    plot(X_pl_Record(1,:),X_pl_Record(2,:),'--r','LineWidth',linewidth);
    set(gca, 'FontSize', 14);
    hold on
    plot(ref.x_ref(end),ref.y_ref(end),'-s','MarkerSize',10,...
        'MarkerEdgeColor','blue','MarkerFaceColor','blue');
    set(gca, 'FontSize', 14);
    hold on
    plot(ref.x_ref(1),ref.y_ref(1),'-s','MarkerSize',10,...
        'MarkerEdgeColor','green','MarkerFaceColor','green');
    set(gca, 'FontSize', 14);
    hold off
    legend('Reference','Feedback','End Point','Start Point',Location='best',FontSize=15);
    title("Path Tracking",FontSize=15)
    xlabel('X coordinate (m)',FontSize=15) 
    ylabel('Y coordinate (m)',FontSize=15) 
    nexttile
    plot(ref.t_ref,ref.psi_ref(1:len),'-k','LineWidth',linewidth);
    set(gca, 'FontSize', 14);
    hold on
    plot(t',X_pl_Record(3,:),'--r','LineWidth',linewidth);
    hold off
    title("Orientation Tracking",FontSize=15)
    legend('Reference','Feedback',FontSize=15);
     xlabel('Time (s)',FontSize=15) 
    ylabel('Yaw (rad)',FontSize=15) 
    nexttile
    plot(ref.t_ref,ref.v_ref(1:len).*ref.curvature_ref(1:len),'-k','LineWidth',linewidth);
    set(gca, 'FontSize', 14);
    hold on
    plot(t',X_pl_Record(6,:),'--r','LineWidth',linewidth);
    if ekf_switch
    hold on
    plot(ref.t_ref,X_hat_Record(2,:),'LineWidth',linewidth);
    end
    hold off
    title("YawRate Tracking",FontSize=15)
    if ekf_switch
        legend('Reference','Feedback','Estimate',FontSize=15)
    else
        legend('Reference','Feedback',FontSize=15);
    end
    xlabel('Time (s)',FontSize=15) 
    ylabel('Yaw Rate (rad/s)',FontSize=15) 
    
    % nexttile
    % plot(t(1:end-1),U_Record(1,:))
    % title("Acceleration Command")
    % nexttile
    % plot(t(1:end-1),rad2deg(U_Record(4,:)))
    % title("Steering Command (deg)")
    
    if ekf_switch
        figure(2)
        tiledlayout("vertical")
        nexttile
        plot(ref.t_ref,plant.C_f*ones(length(ref.t_ref),1),'-k','LineWidth',2);
        hold on
        plot(ref.t_ref,X_hat_Record(5,1)*ones(length(ref.t_ref),1),'--r','LineWidth',2);
        hold on
        plot(ref.t_ref,X_hat_Record(5,:),'-*b','LineWidth',1);
        title("Front Cornering Stiffness")
        legend('Actual (static)','Assumed','Estimated');
        xlabel('Time (s)') 
        ylabel('Cornering Stiffness (N/rad)') 
    
        nexttile
        plot(ref.t_ref,plant.C_r*ones(length(ref.t_ref),1),'-k','LineWidth',2);
        hold on
        plot(ref.t_ref,X_hat_Record(6,1)*ones(length(ref.t_ref),1),'--r','LineWidth',2);
        hold on
        plot(ref.t_ref,X_hat_Record(6,:),'-*b','LineWidth',1);
        title("Rear Cornering Stiffness")
        legend('Actual (static)','Assumed','Estimated');
        xlabel('Time (s)') 
        ylabel('Yaw Rate (rad/s)') 
    end
    if mrac_switch
        figure(3)
        plot(ref.t_ref(1:end-1)',gains_MRAC_vec(1,:),'LineWidth',2);
        set(gca, 'FontSize', 14);
        hold on
        plot(ref.t_ref(1:end-1)',gains_MRAC_vec(2,:),'LineWidth',2);
        set(gca, 'FontSize', 14);
        hold on
        plot(ref.t_ref(1:end-1)',gains_MRAC_vec(3,:),'LineWidth',2);
        set(gca, 'FontSize', 14);
        hold on
        plot(ref.t_ref(1:end-1)',gains_MRAC_vec(4,:),'LineWidth',2);
        set(gca, 'FontSize', 14);
        hold on
        plot(ref.t_ref(1:end-1)',gains_MRAC_vec(5,:),'LineWidth',2);
        set(gca, 'FontSize', 14);
        hold on
        plot(ref.t_ref(1:end-1)',gains_MRAC_vec(6,:),'LineWidth',2);
        set(gca, 'FontSize', 14);
        hold on
        plot(ref.t_ref(1:end-1)',gains_MRAC_vec(7,:),'LineWidth',2);
        set(gca, 'FontSize', 14);
        hold on
        plot(ref.t_ref(1:end-1)',gains_MRAC_vec(8,:),'LineWidth',2);
        set(gca, 'FontSize', 14);
        hold on
        plot(ref.t_ref(1:end-1)',gains_MRAC_vec(9,:),'LineWidth',2);
        set(gca, 'FontSize', 14);
        hold off
        title("Adaptive Gains",FontSize=14)
        legend('Feedforward','State Feedback - $e_y$','State Feedback - $\dot{e}_y$',...
            'State Feedback - $e_\psi$','State Feedback - $\dot{e}_\psi$','Error Feedback - $e_y$',...
            'Error Feedback - $\dot{e}_y$','Error Feedback -  $e_\psi$','Error Feedback - $\dot{e}_y$','interpreter','latex',FontSize=14);
        xlabel('Time of Scenario (s)','FontSize', 15) 
        ylabel('Gain Value','FontSize', 15)
    end
    
    figure(4)
    tiledlayout(2,1)
    nexttile
    plot(run_time_lon,'LineWidth',2);
    set(gca, 'FontSize', 14);
    title("Lon MPC Time",'FontSize', 15)
    xlabel('Sample','FontSize', 15) 
    ylabel('Solve Time (s)','FontSize', 15)
    ylim([0 1.4*(10^-3)])
    nexttile
    plot(run_time_lat,'LineWidth',2);
    set(gca, 'FontSize', 14);
    xlabel('Sample','FontSize', 15) 
    ylabel('Solve Time (s)','FontSize', 15) 
    title("Lat MRAC Time",'FontSize', 15)

    figure(5)
    plot(lat_error.Ey);
    title("Lateral Tracking Error")