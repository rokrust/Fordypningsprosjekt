function [ dr_tot ] = calculate_corrections( base_pos, data)
    n = size(data.pseudorange, 1);
    dr_tot = zeros(32, n);
    
    for i = 1:n
    
        ind = data.pseudorange(i, :) ~= 0;    
        sat_poss = squeeze(data.satPos(i, :, :))';
        pr = data.pseudorange(i, :);
    
        r = sqrt(sum((sat_poss - base_pos) .^ 2)); %Calculate true range
        dr = r - pr;
        %dr(~ind) = deal(0);      %Set non-visible satellite correction to zero
        dr_tot(:, i) = dr';
    end
    dr = dr_tot;
    t_b = data.t;
    save('pr_corr.mat', 'dr', 't_b')
end

