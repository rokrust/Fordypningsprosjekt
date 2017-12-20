function [ d_pr ] = sagnac_correction( pos, sat_pos )
    if pos(1) == 0 || pos(2) == 0
        n = size(sat_pos, 2)
        d_pr = zeros(1, n);
        return
    end

    w_ie = 7.292115*10^(-5);
    c = 299792458;

    d_pr = w_ie*(pos(2)*sat_pos(1, :) - pos(1)*sat_pos(2, :))/(c);
end

