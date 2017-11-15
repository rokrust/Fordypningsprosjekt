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
    
         %{
    sys = ss(ekf.A, ekf.B, zeros(n_sat, n), zeros(n_sat, size(ekf.B, 2)));
    sys = c2d(sys, h);     
    ekf.A = sys.A;
    ekf.B = sys.B;
        %}
    ekf.x_hat = zeros(n, 1);
    ekf.Q = [   eye(dim)        zeros(dim)      zeros(dim, 1)   zeros(dim, 1)
                zeros(dim)      eye(dim)        zeros(dim, 1)   zeros(dim, 1);   
                zeros(1, dim)   zeros(1, dim)   4*10^-19        0;
                zeros(1, dim)   zeros(1, dim)   0               32*(pi^2)*(10^(-20))];
    ekf.Q = eye(n);        
    %ekf.Q = ((ekf.A) / ekf.A)*ekf.Q;
            
    ekf.R = eye(n_sat);
    ekf.P = eye(n);
end