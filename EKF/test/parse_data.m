%{
function [pr, sat_pos, n_sat] = parse_data(data, i)
    vis_sat = data.pseudorange(i, :) ~= 0;           %visible satellites index (EL not 0)
    n_sat = sum(vis_sat);                            %number of visible satellites
    sat_pos = squeeze(data.satPos(i, vis_sat, :))';   %Satellite positions
    pr = data.pseudorange(i, vis_sat)';               %Satellite pseudoranges
end
%}

function [pr, sat_pos] = parse_data(data, i, dr, dgps)
    vis_sat = data.pseudorange(i, :) ~= 0;           %visible satellites index (EL not 0)
    vis_sat(4) = 0;
    sat_pos = squeeze(data.satPos(i, vis_sat, :))';   %Satellite positions
    
    if dgps
        
        pr = data.pseudorange(i, :) + dr(:, i)';
    else
        c = 299792458.0;
        d_sv = data.sv_clock(i, :) + data.relativistic(i, :);
        pr = data.pseudorange(i, :) + d_sv*c;
    end
    vis_sat(4) = 0;
    pr = pr(vis_sat)';               %Satellite pseudoranges
end
