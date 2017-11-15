function r = range_estimate(sat_pos, x_hat)
    pos = x_hat(1:3);
    n_sat = size(sat_pos, 2);
    
    r = sqrt(sum((sat_pos - pos) .^ 2))';
    
end