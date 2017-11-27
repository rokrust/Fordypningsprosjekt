function [ el ] = satel( pos, sat_pos )
    %http://www.naic.edu/aisr/GPSTEC/drewstuff/MATLAB/elavazim.m
    diff_pos = sat_pos - pos;
    los_vec = diff_pos / norm(diff_pos);
    dx = los_vec(1); dy = los_vec(2); dz = los_vec(3);
    
    lla = ecef2lla(pos');
    lat = deg2rad(lla(1)); lon = deg2rad(lla(2));
    
    vertical = cos(lon)*cos(lat) .* dx-sin(lon)*cos(lat) .* dy-sin(lat) .* dz;
    el = rad2deg(pi/2 - acos(vertical));
    
end

