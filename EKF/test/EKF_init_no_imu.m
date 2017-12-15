function ekf = EKF_init_no_imu()
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
    
    ekf.B = [zeros(dim, 1);
             ones(dim, 1);
             zeros(1, 1);
             zeros(1, 1)];
    
    sys = ss(ekf.A, ekf.B, zeros(1, n), zeros(1, size(ekf.B, 2)));
    sys = c2d(sys, h);     
    ekf.A = sys.A;
    ekf.B = sys.B;
    
    ekf.Q = diag([zeros(1, 3) 0.1*ones(1, 3) 10 10]);
    
    ekf.R  = eye(1);
    ekf.P  = eye(8);
    ekf.P_ = eye(8);
    
    P0 = [0; 0; 0];
    
    ekf.x_hat_  = [P0; zeros(3, 1); 0; 0];
    ekf.x_hat   = [P0; zeros(3, 1); 0; 0];
    ekf.res = 0;
end