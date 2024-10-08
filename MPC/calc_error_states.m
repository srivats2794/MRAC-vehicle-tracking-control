function error_states = calc_error_states(X,Y,psi,psiDot,Vx,Vy,X_ref,Y_ref,psi_ref,kappa_ref)
  X_diff= X-X_ref;
  Y_diff= Y-Y_ref;
  V_total=sqrt(Vx^2 + Vy^2);

  Ey= Y_diff*cos(psi)-X_diff*sin(psi);
  Epsi= psi-psi_ref;
  EyDot= V_total*sin(Epsi);
  EpsiDot= psiDot-V_total*kappa_ref;

  error_states= [Ey;EyDot;Epsi;EpsiDot];
end

