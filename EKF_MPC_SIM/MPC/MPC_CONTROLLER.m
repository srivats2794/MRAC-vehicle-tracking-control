classdef MPC_CONTROLLER
    % This class sets up the linear MPC controller using OSQP solver
    
    properties
       Ts;
       Q;
       R;
       QN;
       N, nx, nu;
       solver, P, A, q, l, u;
       x_min, x_max, u_min,u_max
       sys_params
    end
    
    methods
        function obj = MPC_CONTROLLER(params)
           obj.Ts=params.Ts; obj.N=params.N; obj.Q=params.Q;obj.R=params.R; 
           obj.nx=params.nx; obj.nu=params.nu;
           obj.QN=params.QN; 
           obj.x_min=params.x_min;obj.x_max=params.x_max;
           obj.u_min=params.u_min;obj.u_max=params.u_max;

           obj.P  = blkdiag( kron(speye(obj.N), obj.Q), obj.QN, kron(speye(obj.N), obj.R) );
           xr=zeros(obj.nx,1);
           Ax = kron(speye(obj.N+1), -speye(obj.nx)) + kron(sparse(diag(ones(obj.N, 1), -1)), params.sys_params.A_sys);
           Bu = kron([sparse(1, obj.N); speye(obj.N)], params.sys_params.B_sys);
           obj.q = [repmat(-obj.Q*xr, obj.N, 1); -obj.QN*xr; zeros(obj.N*obj.nu, 1)];
           Aeq    = [Ax, Bu];
           x0     = zeros(obj.nx,1);
           leq           = [-x0; zeros(obj.N*obj.nx, 1)];
           ueq           = leq;
           Aineq = speye((obj.N+1)*obj.nx + obj.N*obj.nu);
           
           lineq         = [repmat(obj.x_min, obj.N+1, 1); repmat(obj.u_min, obj.N, 1)];
           uineq         = [repmat(obj.x_max, obj.N+1, 1); repmat(obj.u_max, obj.N, 1)];

           obj.A             = [Aeq; Aineq];
           obj.l             = [leq; lineq];
           obj.u             = [ueq; uineq];

           obj.sys_params=params.sys_params;

           obj.solver=osqp;
           
           obj.solver.setup(obj.P, obj.q, obj.A, obj.l, obj.u, 'warm_start', true,'verbose',false);
        end
    end
end

