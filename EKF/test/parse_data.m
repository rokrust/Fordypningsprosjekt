%{
function [pr, sat_pos, n_sat] = parse_data(data, i)
    vis_sat = data.pseudorange(i, :) ~= 0;           %visible satellites index (EL not 0)
    n_sat = sum(vis_sat);                            %number of visible satellites
    sat_pos = squeeze(data.satPos(i, vis_sat, :))';   %Satellite positions
    pr = data.pseudorange(i, vis_sat)';               %Satellite pseudoranges
end
%}
%{
function [pr, sat_pos, n_sat] = parse_data(data, i)
    vis_sat = data.EL(:, i) ~= 0;
    n_sat = sum(vis_sat);
    sat_pos = data.Satpos(:, vis_sat, i);
    pr = data.PR(vis_sat, i);

end
%}


function [pr, sat_pos] = parse_data(data, i, dr)
    vis_sat = data.pseudorange(i, :) ~= 0;           %visible satellites index (EL not 0)
    sat_pos = squeeze(data.satPos(i, vis_sat, :))';   %Satellite positions
    pr = data.pseudorange(i, :) + dr.dr(:, i)';
    pr = pr(vis_sat)';               %Satellite pseudoranges
end
