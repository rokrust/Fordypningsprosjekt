function [ I_gps ] = ionospheric_correction( iono_param, el, azi, lat, lon, t_gps )
%IONOSPHERIC_CORRECTION Summary of this function goes here
%   From http://www.navipedia.net/index.php/Klobuchar_Ionospheric_Model
a0 = iono_param(1); a1 = iono_param(2); a2 = iono_param(3); a3 = iono_param(4);
b0 = iono_param(5); b1 = iono_param(6); b2 = iono_param(7); b3 = iono_param(8);


psi = 0.0137 / (el + 0.11) - 0.022;
lat_i = lat + psi*cos(azi);
if abs(lat) > 0.416
    lat_i = 0.416*sign(lat);
end

lon_i = lon + psi*sin(azi)/cos(lat_i);
lat_m = lat_i + 0.064*cos(lon_i - 1.617);

t = 43200*lon_i + t_gps;
if t >= 86400
    t = t - 86400;
elseif t < 0
    t = t + 86400;
end

A_i = a0 + a1*lat_m + a2*(lat_m^2) + a3*(lat_m^3);
P_i = b0 + b1*lat_m + b2*(lat_m^2) + b3*(lat_m^3);

if A_i < 0
    A_i = 0;
end
if P_i < 72000
    P_i = 72000;
end

X_i = 2*pi(t - 50400)/P_i;
F = 1.0 + 16.0*(0.53 - el)^3;

I_gps = 5*10^-9;
if abs(X_i) <= 1.57
    I_gps = I_gps + A_i * (1 - X_i^2/2 + X_i^4/24)*F;
else
    I_gps = I_gps*F;
end

end

