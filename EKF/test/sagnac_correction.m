function [ d_pr ] = sagnac_correction( pos, sat_pos )
if pos(1) == 0 || pos(2) == 0
    d_pr = 0;
    return
end

w_ie = 7.292115*10^(-5);
c = 299792458;

d_pr = w_ie./(c*(pos(1)*sat_pos(2, :) - pos(2)*sat_pos(1, :)));


end

