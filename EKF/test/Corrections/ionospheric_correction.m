function [ I_gps ] = ionospheric_correction( iono_param, el, azi, lat, lon, t_gps )
%   From http://www.navipedia.net/index.php/Klobuchar_Ionospheric_Model
    a0 = iono_param(1); a1 = iono_param(2); a2 = iono_param(3); a3 = iono_param(4);
    b0 = iono_param(5); b1 = iono_param(6); b2 = iono_param(7); b3 = iono_param(8);

    %Convert to semi-circles
    lat_u = deg2rad(lat);
    lon_u = deg2rad(lon);

    azi_r = deg2rad(azi);
    el_r = deg2rad(el);

    psi = 0.0137 ./ (el_r./pi + 0.11) - 0.022;
    lat_i = lat_u./pi + psi .* cos(azi_r); %phi
    
    %Saturate
    ind = abs(lat_i) > 0.416;
    lat_i(ind) = 0.416*sign(lat_i(ind));
    
    %lon_i = lon_u + psi.*sin(azi_r)./cos(lat_i);
    lat_m = lon_u./pi + psi.*sin(azi_r)./cos(lat_i*pi);
    lat_i = lat_i + 0.064.*cos((lat_m - 1.617)*pi);
    
    
    %Time of intersection
    
    t = 43200*lat_m + t_gps;
    t = t - floor(t/86400.0)*86400.0;
    
    %{
    ind = t >= 86400;
    while any(ind)
        t(ind) = t(ind) - 86400;
        ind = t >= 86400;
    end
    
    ind = t < 0;
    while any(ind)
        t(ind) = t(ind) + 86400;
        ind = t < 0;
    end
    %}
    A_i = a0 + a1*lat_i + a2*(lat_i.^2) + a3*(lat_i.^3);  %Iono delay amplitude
    P_i = b0 + b1*lat_i + b2*(lat_i.^2) + b3*(lat_i.^3);  %iono delay phase

    %Saturate
    ind = A_i < 0;
    A_i(ind) = 0;

    %Saturate
    ind = P_i < 72000;
    P_i(ind) = 72000;

    X_i = 2*pi*(t - 50400) ./ P_i;
    F = 1.0 + 16.0 .* (0.53 - el_r/pi) .^ 3.0;
    
    n = size(X_i, 2);
    I_gps = 5*10^-9*ones(1, n);
    ind = abs(X_i) < 1.57;
    
    I_gps(ind) = F .* (I_gps(ind) + A_i .* (1 - (X_i .^ 2)/2.0 + (X_i .^ 4)/24));
end

