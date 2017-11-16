function G = geometry_matrix_no_imu(x_hat, sat_poss, pseu)
    n_sat = size(pseu, 1);
    dim = size(sat_poss, 1);
    
    G = [(x_hat(1:3) - sat_poss)' ./ pseu, zeros(n_sat, dim)];
    G = [G ones(n_sat, 1), zeros(n_sat, 1)];
end