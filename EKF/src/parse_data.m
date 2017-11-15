function [pr, sat_pos, n_sat] = parse_data(data, i)
    vis_sat = data.EL(:, i) ~= 0;           %visible satellites index (EL not 0)
    n_sat = sum(vis_sat);                   %number of visible satellites
    sat_pos = data.Satpos(:, vis_sat, i);   %Satellite positions
    pr = data.PR(vis_sat, i);               %Satellite pseudoranges
end