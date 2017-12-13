function [ dr ] = calculate_corrections( base_pos, data, bias, i)
%CALCULATE_CORRECTIONS Summary of this function goes here
%   Detailed explanation goes here
    
    m = [-24.3405487029696,0,-4.59257180039419,-14.5333432317667,0,0,0,-27.6333212851855,0,-26.6159662506160,-29.9409669325759,0,0,-27.9285350122082,0,0,-0.175563750020363,-22.4907656904563,0,0,0,-28.1274660000990,0,-18.2596758408735,0,0,-24.0560768548614,-29.2310960120253,0,0,0,-27.5833916052421];
    
    ind = data.pseudorange(i, :) ~= 0;    
    sat_poss = squeeze(data.satPos(i, :, :))';
    pr = data.pseudorange(i, :);
    
    r = sqrt(sum((sat_poss - base_pos) .^ 2)); %Calculate true range
    dr = r - (pr + bias);
    
    trans = abs(dr-m) > 10;
    dr(trans) = m(trans);
    dr(~ind) = deal(0);      %Set non-visible satellite correction to zero
    dr = dr';
end

