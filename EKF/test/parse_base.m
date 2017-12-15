function [pr, sat_pos] = parse_data(data, i)
    c = 299792458.0;

    vis_sat = data.pseudorange(i, :) ~= 0;           %visible satellites index (EL not 0)
    vis_sat(4) = 0;
    sat_pos = squeeze(data.satPos(i, vis_sat, :))';   %Satellite positions
    d_pr = (data.sv_clock(i, :) + data.relativistic(i, :))*c;
    pr = data.pseudorange(i, :) + d_pr;
    pr = pr(vis_sat)';
end