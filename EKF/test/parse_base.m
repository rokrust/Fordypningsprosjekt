function [pr, sat_pos] = parse_data(data, i)
    vis_sat = data.pseudorange(i, :) ~= 0;           %visible satellites index (EL not 0)
    sat_pos = squeeze(data.satPos(i, vis_sat, :))';   %Satellite positions
    pr = data.pseudorange(i, vis_sat)';               %Satellite pseudoranges
end