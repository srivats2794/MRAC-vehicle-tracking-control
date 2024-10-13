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
    title("MPC Path Tracking",FontSize=15)
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
    ylabel('MPC Yaw (rad)',FontSize=15) 
    nexttile
    plot(ref.t_ref,ref.v_ref(1:len).*ref.curvature_ref(1:len),'-k','LineWidth',linewidth);
    set(gca, 'FontSize', 14);
    hold on
    plot(t',X_pl_Record(6,:),'--r','LineWidth',linewidth);
    hold off
    title("MPC YawRate Tracking",FontSize=15)
    
        legend('Reference','Feedback',FontSize=15);
    xlabel('Time (s)',FontSize=15) 
    ylabel('Yaw Rate (rad/s)',FontSize=15) 
   
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
    title("Lat MPC Time",'FontSize', 15)
    % 
    % figure(5)
    % plot(lat_error.Ey);
    % title("Lateral Tracking Error")