function G = geometry_matrix(x_hat, sat_poss, pseu)
    n_sat = size(sat_poss, 1);
    G = [(sat_poss - x_hat(1:3)') ./ pseu, ones(n_sat, 1)];
end