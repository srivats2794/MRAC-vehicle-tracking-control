function params=updateLonMpc(vx,Ax,vx_ref,vy,psiDot,lon_mpc)
    distTerm = -[lon_mpc.Ts;0]*(vy*psiDot);
    xr(1:2,1:lon_mpc.N)= [-lon_mpc.Q(1,1)*vx_ref(1:lon_mpc.N)';-lon_mpc.Q(2,2)*zeros(1,lon_mpc.N)];
    xr(1:2,lon_mpc.N+1)= -lon_mpc.QN*[vx_ref(lon_mpc.N+1);0];
    
    x0= [vx;Ax];
    params.q = [xr(:); zeros(lon_mpc.N*lon_mpc.nu, 1)];
    leq           = [-x0+distTerm; zeros(lon_mpc.N*lon_mpc.nx, 1)];
    ueq           = leq;
    lineq = [repmat(lon_mpc.x_min, lon_mpc.N+1, 1); repmat(lon_mpc.u_min, lon_mpc.N, 1)];
    uineq = [repmat(lon_mpc.x_max, lon_mpc.N+1, 1); repmat(lon_mpc.u_max, lon_mpc.N, 1)];
    params.l = [leq; lineq];
    params.u = [ueq; uineq];
end
