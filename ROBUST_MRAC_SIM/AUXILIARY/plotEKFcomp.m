load("EKFdata.mat");
load("NoEKFdata.mat");

figure(1)

nexttile
plot(EKF.t,EKF.Vx_ref);
hold on
plot(EKF.t,EKF.Vx);
hold on
plot(NoEKF.t,NoEKF.Vx)
hold off
title("Velocity Tracking")
legend('Reference','EKF','W/O EKF');

nexttile
plot(EKF.X_ref,EKF.Y_ref);
hold on
plot(EKF.X,EKF.Y);
hold on
plot(EKF.X_ref(1),EKF.Y_ref(1),'-s','MarkerSize',10,...
    'MarkerEdgeColor','red');
hold on
plot(EKF.X_ref(end),EKF.Y_ref(end),'-s','MarkerSize',10,...
    'MarkerEdgeColor','green');
hold on
plot(NoEKF.X,NoEKF.Y);
hold off
legend('Reference','EKF','End Point','Start Point','W/O EKF',Location='best');
title("Path Tracking")

nexttile
plot(EKF.t,EKF.psi_ref);
hold on
plot(EKF.t,EKF.psi);
hold on
plot(NoEKF.t,NoEKF.psi);
hold off
title("Orientation Tracking")
legend('Reference','EKF','W/O EKF');

nexttile
plot(EKF.t,EKF.Vx_ref.*EKF.kappa_ref);
hold on
plot(EKF.t,EKF.psiDot);
hold on
plot(NoEKF.t,NoEKF.psiDot);
hold off
title("YawRate Tracking")
legend('Reference','EKF','W/O EKF');

nexttile
plot(EKF.t,rad2deg(EKF.delta_cmd))
hold on
plot(NoEKF.t,rad2deg(NoEKF.delta_cmd))
hold off
title("Steering Command (deg)")
legend('EKF','W/O EKF');

    figure(2)
    nexttile
    plot(EKF.t,EKF.C_f,'-k','LineWidth',2);
    hold on
    plot(EKF.t,EKF.C_f_init*ones(length(EKF.t),1),'--r','LineWidth',2);
    hold on
    plot(EKF.t,EKF.C_f_est,'-*b','LineWidth',2);
    hold off
    title("Front Cornering Stiffness")
    legend('Actual','Assumed','Estimated');

    nexttile
    plot(EKF.t,EKF.C_r,'-k','LineWidth',2);
    hold on
    plot(EKF.t,EKF.C_r_init*ones(length(EKF.t),1),'--r','LineWidth',2);
    hold on
    plot(EKF.t,EKF.C_r_est,'-*b','LineWidth',2);
    hold off
    title("Rear Cornering Stiffness")
    legend('Actual','Assumed','Estimated');

    lat_error_EKF= calc_LateralDeviation(EKF.X,EKF.Y,EKF.X_ref,EKF.Y_ref,EKF.psi,0.01,0.01);
    lat_error_NoEKF= calc_LateralDeviation(NoEKF.X,NoEKF.Y,NoEKF.X_ref,NoEKF.Y_ref,NoEKF.psi,0.01,0.01);