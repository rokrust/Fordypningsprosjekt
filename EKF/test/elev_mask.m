function [ pr_o, sat_poss_o, el_o, azi_o ] = elev_mask( pr, sat_poss, el, azi, min_elev)
    if sum(el >= min_elev) >= 4
        ind = el < min_elev;
        el(ind) = [];
        azi(ind) = [];
        pr(ind) = [];
        sat_poss(:, ind) = [];
    end
    
    pr_o = pr; sat_poss_o = sat_poss; el_o = el; azi_o = azi;
end

