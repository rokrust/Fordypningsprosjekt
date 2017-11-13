function [x_hat, P] = EKF_step(sat_poss, pr, u, ekf)
    %%Validity checks
    
    %%Initial variables
    n_sat = size(sat_poss, 1);
    n = n_sat + ekf.n_const;
    
    %%Kalman algorithm
    G = geometric_matrix(x_hat, sat_poss, pr);
    ekf.H = [zeros(n_const), G]
    
    K = 

end

%x_dot = Ax + Bu
%x = [p; v; theta; b_a; b_w; b_c; pseu(..)]';
%u = [a, w];