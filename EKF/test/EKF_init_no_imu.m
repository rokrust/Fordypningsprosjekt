function ekf = EKF_init_no_imu(n_sat)
    n = 8;
    dim = 3;
    c = 299792458;
    h = 2^-1;
    
    ekf.cfg.dim = dim;
    ekf.cfg.n = n; %pos, vel, ang, bias
    ekf.cfg.h = h;
    ekf.cfg.c = c;

    ekf.h = @geometry_matrix_no_imu;                       %Measure function
    ekf.A = [zeros(dim)     eye(dim)        zeros(dim, 1)   zeros(dim, 1);
             zeros(dim)     zeros(dim)      zeros(dim, 1)   zeros(dim, 1);
             zeros(1, dim)  zeros(1, dim)   0               1;
             zeros(1, dim)  zeros(1, dim)   0               0];

    ekf.B = [zeros(dim)     zeros(dim, 1)   zeros(dim, 1);
             eye(dim)       zeros(dim, 1)   zeros(dim, 1);
             zeros(1, dim)  c               0;
             zeros(1, dim)  0               c];
    
    sys = ss(ekf.A, ekf.B, zeros(n_sat, n), zeros(n_sat, size(ekf.B, 2)));
    sys = c2d(sys, h);     
    ekf.A = sys.A;
    ekf.B = sys.B;
    
    ekf.Q = diag([zeros(1, 3) 0.1*ones(1, 3) 10 10]);
    %ekf.Q(1:3, 1:3) = zeros(3);
    ekf.Q = ekf.Q;
    
    ekf.R  = eye(n_sat);
    ekf.P  = eye(8);
    ekf.P_ = eye(8);
    
    P0 = [2.799832e+06; 4.79942e+05; 5.691477e+06];
    %P0 = [4060565.23310000;608399.449100000;4864639.90330000];
    P0 = [0; 0; 0];
    
    ekf.x_hat_  = [P0; zeros(3, 1); 0; 0];
    ekf.x_hat   = [P0; zeros(3, 1); 0; 0];
    ekf.res = 0;
end