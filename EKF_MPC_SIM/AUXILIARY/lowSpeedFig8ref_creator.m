t_vec= out.Time.Time; 
a_ref= zeros(length(EKF.t),1);
a_ref(2:end,1)= diff(EKF.Vx_ref)./diff(EKF.t);
ref.x_ref=makima(EKF.t,EKF.X_ref,t_vec);
        ref.y_ref=makima(EKF.t,EKF.Y_ref,t_vec);
        ref.psi_ref=makima(EKF.t,EKF.psi_ref,t_vec);
        ref.v_ref= makima(EKF.t,EKF.Vx_ref,t_vec);
        ref.curvature_ref=makima(EKF.t,EKF.kappa_ref,t_vec);
        ref.a_ref=makima(EKF.t,a_ref,t_vec);
        ref.t_ref=t_vec;


out.Reference.signal_264__X_Position_ReferencePath_global__m_.Data= ref.x_ref;
out.Reference.signal_265__Y_Position_ReferencePath_global__m_.Data=ref.y_ref;
out.Reference.signal_253__Target_Speed_For_Default_Controller__km_h_.Data= ref.v_ref/0.277778;
out.KinematicState.signal_3D_Rotations_RA.signal_80__Yaw_Angle__deg_.Data= rad2deg(ref.psi_ref);
out.Reference.signal_255__Curvature_Reference_Default_Controller__m__1_.Data=ref.curvature_ref;