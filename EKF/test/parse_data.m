%{
function [pr, sat_pos] = parse_data(data, i, corr_data, dgps, bias)
    vis_sat = data.pseudorange(i, :) ~= 0;           %visible satellites index (EL not 0)
    vis_sat(4) = 0;
    sat_pos = squeeze(data.satPos(i, vis_sat, :))';   %Satellite positions
    
    if dgps
        t_r = data.t(i);
        
        if i > 100
            t_r = t_r + bias/299792458.0;
        end
        
        time_diff_prev = Inf;
        
        %Match rover and base time
        j = 1;
        for j = 30:size(corr_data.t_corr,2)
            time_diff_curr = abs(t_r - corr_data.t_corr(j));
            
            % Best match found
            if time_diff_curr > time_diff_prev
                break
            end
            
            time_diff_prev = time_diff_curr;
        end

        pr = data.pseudorange(i, :) - corr_data.dr.dr(:, j-1)';
    else
        c = 299792458.0;
        d_sv = data.sv_clock(i, :) + data.relativistic(i, :);
        pr = data.pseudorange(i, :) + d_sv*c;
    end
    vis_sat(4) = 0;
    pr = pr(vis_sat)';               %Satellite pseudoranges
end
%}

function [pr, sat_pos] = parse_data(data, i, corr_data, dgps, bias)
    vis_sat = data.pseudorange(i, :) ~= 0;           %visible satellites index (EL not 0)
    vis_sat(4) = 0;
    sat_pos = squeeze(data.satPos(i, vis_sat, :))';   %Satellite positions
    
    if dgps
        t_r = data.t(i);       
        time_diff_prev = Inf;
        
        %Match rover and base time
        j = 1;
        for j = 30:size(corr_data.t_b,2)
            time_diff_curr = abs(t_r - corr_data.t_b(j));
            
            % Best match found
            if time_diff_curr > time_diff_prev
                break
            end
            
            time_diff_prev = time_diff_curr;
        end

        pr = data.pseudorange(i, :) + corr_data.dr(:, j-1)';
    else
        c = 299792458.0;
        d_sv = data.sv_clock(i, :) + data.relativistic(i, :);
        pr = data.pseudorange(i, :) + d_sv*c;
    end
    vis_sat(4) = 0;
    pr = pr(vis_sat)';               %Satellite pseudoranges
end

