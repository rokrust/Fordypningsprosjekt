function G = geometry_matrix(x_hat, sat_poss, pseu)
    G = [(sat_poss - x_hat(1:3))' ./ pseu];
end