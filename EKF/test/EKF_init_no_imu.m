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
    
    ekf.err = cell(n_sat);
    
    ekf.Q = diag([zeros(1, 3) zeros(1, 3) 0.1 0.1]);
    %{
    ekf.Q = [   zeros(dim)        zeros(dim)      zeros(dim, 1)   zeros(dim, 1)
                zeros(dim)      eye(dim)        zeros(dim, 1)   zeros(dim, 1);   
                zeros(1, dim)   zeros(1, dim)   4*10^-19        0;
                zeros(1, dim)   zeros(1, dim)   0               32*(pi^2)*(10^(-20))];   
    %}
    ekf.R = eye(n_sat);
    ekf.P       = eye(n);
    ekf.P_      = [2.71478293012976,0.347255573622875,-0.377423717005220,2.16993238370062,0.168610486137210,-0.176798620684704,0.000729640170522256,9.88733040833084e-08;0.347255573622875,3.12368921308823,-0.331262727130438,0.168610495271924,2.37358237949695,-0.150687281689366,0.000427871267723673,5.79806968373343e-08;-0.377423717005220,-0.331262727130438,3.89658543420856,-0.176800220049766,-0.150688006306866,2.73130458023529,0.00129087250165019,1.74925716431832e-07;2.16993238370062,0.168610495271924,-0.176800220049766,3.48424541323814,0.108863097106629,-0.110249184564445,8.26206893427693e-08,1.63053858895306e-11;0.168610486137210,2.37358237949695,-0.150688006306866,0.108863097106629,3.61834748516024,-0.0914719712236165,8.68763941356235e-08,1.47680541901541e-11;-0.176798620684704,-0.150687281689366,2.73130458023529,-0.110249184564445,-0.0914719712236165,3.83986575157083,1.46218093117443e-07,2.88537687785502e-11;0.000729640170522256,0.000427871267723673,0.00129087250165019,8.26206893427693e-08,8.68763941356235e-08,1.46218093117443e-07,0.000991960996106151,1.34420314283514e-07;9.88733040833084e-08,5.79806968373343e-08,1.74925716431832e-07,1.63053858895306e-11,1.47680541901541e-11,2.88537687785502e-11,1.34420314283514e-07,2.51613106954068e-11];
    
    P0 = [2.799832e+06; 4.79942e+05; 5.691477e+06];
    ekf.x_hat_  = [P0; zeros(3, 1); 0; 0];
    ekf.x_hat   = [P0; zeros(3, 1); 0; 0];
end