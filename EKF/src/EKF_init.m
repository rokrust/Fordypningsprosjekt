function ekf = EKF_init(n_sat)
    n = 13;
    dim = 3;
    h = 0.1;
    ekf.cfg.dim = dim;
    ekf.cfg.n = n; %pos, vel, ang, bias

    T_w = [0.5 0.5 0.5];

    ekf.h = @geometry_matrix; %Measure function
    ekf.A = [zeros(dim)    eye(dim)      zeros(dim)     zeros(dim)    zeros(dim, 1);
             zeros(dim)    zeros(dim)    zeros(dim)     zeros(dim)    zeros(dim, 1);
             zeros(dim)    zeros(dim)    zeros(dim)     -eye(dim)     zeros(dim, 1);
             zeros(dim)    zeros(dim)    zeros(dim)     -diag(T_w)    zeros(dim, 1);
             zeros(1, dim) zeros(1, dim) zeros(1, dim)  zeros(1, dim) 0           ];

    ekf.B = [zeros(dim)     zeros(dim);
             eye(dim)       zeros(dim);
             zeros(dim)     eye(dim);
             zeros(dim)     zeros(dim)
             zeros(1, dim)  zeros(1, dim)];
    
    %{
    sys = ss(ekf.A, ekf.B, zeros(n_sat, n), zeros(n_sat, size(ekf.B, 2)));
    sys = c2d(sys, h);     
    ekf.A = sys.A;
    ekf.B = sys.B;
    %}
             
    ekf.x_hat = zeros(n, 1);
    ekf.Q = eye(n);
    ekf.R = eye(n_sat);
    ekf.P = eye(n);
end