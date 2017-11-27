function [pr, sat_pos, n_sat] = parse_data(data, i)
    vis_sat = data.pseudorange(i, :) ~= 0;           %visible satellites index (EL not 0)
    n_sat = sum(vis_sat);                   %number of visible satellites
    sat_pos = reshape(data.satPos(i, vis_sat, :), [n_sat, 3])';   %Satellite positions
    pr = data.pseudorange(i, vis_sat)';               %Satellite pseudoranges
end