function G = geometry_matrix(x_hat, sat_poss, pseu)
    G = [(x_hat(1:3) - sat_poss)' ./ pseu];
end